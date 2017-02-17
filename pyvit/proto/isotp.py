import time
from multiprocessing import Queue
from queue import Empty
from enum import Enum

from .. import can

class Isotp_Mtype(Enum):
    diagnostic = 1
    remote_diagnostic = 2

class Isoyp_N_TAtype(Enum):
    physical   = 1
    functional = 2

class Isotp_N_Result(Enum):
    N_OK             = 1
    N_TIMEOUT_A      = 2
    N_TIMEOUT_Bs     = 3
    N_TIMEOUT_Cr     = 4
    N_WRONG_SN       = 5
    N_INVALID_FS     = 6
    N_UNEXP_PDU      = 7
    N_WFT_OVRN       = 8
    N_BUFFER_OVFLW   = 9
    N_ERROR          = 10


class Isotp_Error(Exception):
    def __init__(self,Isotp_N_Result):
        super(Isotp_Error,self).__init__("ISOTP Error " + repr(Isotp_N_Result))
        self.Isotp_N_Result = Isotp_N_Result

class IsotpN_USDataRequest:
    def __init__(self, Mtype,N_SA,N_TA,N_TAtype,data,N_AE=None):
        self._Mtype = Mtype
        self.N_SA       =   N_SA
        self.N_TA       =   N_TA
        self.N_TAtype   =   N_TAtype
        self.data       =   data
        self.N_AE       =   N_AE

class IsotpN_USDataConfirm:
    def __init__(self, IsotpN_USDataRequest, N_Result=Isotp_N_Result.N_OK):
        self._IsotpN_USDataRequest = IsotpN_USDataRequest
        self.N_Result = N_Result

class IsotpN_USDataFFIndication:
    def __init__(self, Mtype,N_SA,N_TA,N_TAtype,length,N_AE=None):
        self._Mtype = Mtype
        self.N_SA       =   N_SA
        self.N_TA       =   N_TA
        self.N_TAtype   =   N_TAtype
        self.length     =   length
        self.N_AE       =   N_AE

class IsotpN_USDataIndication:
    def __init__(self, Mtype,N_SA,N_TA,N_TAtype,data,N_Result,N_AE=None):
        self._Mtype = Mtype
        self.N_SA       =   N_SA
        self.N_TA       =   N_TA
        self.N_TAtype   =   N_TAtype
        self.data       =   data
        self.N_Result   =   N_Result
        self.N_AE       =   N_AE


class IsotpInterface:
    debug = False

    def __init__(self, dispatcher, padding=0,debug=False):
        self._dispatcher = dispatcher
        self.padding_value = padding
        self._recv_queue = Queue()
        self._debug=debug
        self._dispatcher.add_receiver(self._recv_queue)

    def _pad_data(self, data):
        # pad data to 8 bytes
        return data + ([self.padding_value] * (8 - len(data)))

    def _start_msg(self):
        # initialize reading of a message
        self.data = []
        self.data_len = 0
        self.data_byte_count = 0
        self.sequence_number = 0

    def _end_msg(self):
        # finish reading a message
        tmp = self.data
        self.data = []
        self.data_len = 0
        return tmp

    def reset(self):
        # abort reading a message
        self.data = []
        self.data_len = 0

    def parse_frame(self, frame, tx_arb_id):
        # pci type is upper nybble of first byte
        pci_type = (frame.data[0] & 0xF0) >> 4

        if pci_type == 0:
            # single frame

            self._start_msg()

            # data length is lower nybble for first byte
            sf_dl = frame.data[0] & 0xF

            # check that the data length is valid for a SF
            if not (sf_dl > 0 and sf_dl < 8):
                raise ValueError('invalid SF_DL parameter for single frame')

            self.data_len = sf_dl

            # get data bytes from this frame
            self.data = frame.data[1:sf_dl+1]

            # single frame, we're done!
            return self._end_msg()

        elif pci_type == 1:
            # first frame

            self._start_msg()

            # data length is lower nybble of byte 0 and byte 1
            ff_dl = ((frame.data[0] & 0xF) << 8) + frame.data[1]

            self.data_len = ff_dl

            # retrieve data bytes from first frame
            for i in range(2, min(ff_dl+2, 8)):
                self.data.append(frame.data[i])
                self.data_byte_count = self.data_byte_count + 1

            self.sequence_number = self.sequence_number + 1

            # send a flow control frame
            fc = can.Frame(tx_arb_id, data=[0x30, self.bs, self.stmin])
            self._dispatcher.send(fc)
            self.bs_counter = self.bs

        elif pci_type == 2:
            # consecutive frame

            # check that a FF has been sent
            if self.data_len == 0:
                raise ValueError('consecutive frame before first frame')

            # frame's sequence number is lower nybble of byte 0
            frame_sequence_number = frame.data[0] & 0xF

            # check the sequence number
            if frame_sequence_number != self.sequence_number:
                raise ValueError('invalid sequence number!')

            bytes_remaining = self.data_len - self.data_byte_count

            # grab data bytes from this message
            for i in range(1, min(bytes_remaining, 7) + 1):
                self.data.append(frame.data[i])
                self.data_byte_count = self.data_byte_count + 1

            if self.data_byte_count == self.data_len:
                return self._end_msg()
            elif self.data_byte_count > self.data_len:
                raise ValueError('data length mismatch')

            # wrap around when sequence number reaches 0xF
            self.sequence_number = self.sequence_number + 1
            if self.sequence_number > 0xF:
                self.sequence_number = 0

            if self.bs_counter > 0:
                self.bs_counter -= 1
                if self.bs_counter == 0:
                    #Has to send flow control
                    self.bs_counter = self.bs
                    fc = can.Frame(tx_arb_id, data=[0x30, self.bs, self.stmin])
                    self._dispatcher.send(fc)

        else:
            raise ValueError('invalid PCItype parameter')

    def recv(self, rx_arb_id, tx_arb_id, timeout=1,bs=0,stmin=0):
        data = None
        start = time.time()

        self.bs=bs
        if ( not(stmin <= 0x7F)
                and not(stmin >= 0xF1 and stmin<=0xF9)):
           raise ValueError("stmin must be beween 0x00 - 0x7F or 0xF1 - 0XF9")
        self.stmin=stmin

        while data is None:
            # attempt to get data, returning None if we timeout
            try:
                rx_frame = self._recv_queue.get(timeout=timeout)
            except Empty:
                return None

            if self._debug:
                print("ISOTP " + repr(rx_frame.arb_id) + " -> " + repr(rx_frame.data))
            if rx_frame.arb_id == rx_arb_id:
                if self._debug:
                    print("-->ISOTP parse")
                data = self.parse_frame(rx_frame,tx_arb_id)
            else:
                if self._debug:
                    print("-->ISOTP DROP")

            # check timeout, since we may be receiving messages that do not
            # have the required arb_id
            if timeout != None:
                if time.time() - start > timeout:
                    return None

        return data

    def send(self, data, tx_arb_id, rx_arb_id, N_As=1, N_Bs=1,N_Cs=0):
        #TODO use N_as for sending timeout
        if len(data) > 4095:
            raise ValueError('ISOTP data must be <= 4095 bytes long')

        if len(data) < 8:
            # message is less than 8 bytes, use single frame

            sf = can.Frame(tx_arb_id)

            # first byte is data length, remainder is data
            sf.data = self._pad_data([len(data)] + data)

            if self.debug:
                print(sf)
            self._dispatcher.send(sf)

        else:
            # message must be composed of FF and CF

            # first frame
            ff = can.Frame(tx_arb_id)

            frame_data = []
            # FF pci type and msb of length
            frame_data.append(0x10 + (len(data) >> 8))
            # lower byte of data
            frame_data.append(len(data) & 0xFF)
            # first 6 bytes of data
            frame_data = frame_data + data[0:6]

            ff.data = self._pad_data(frame_data)
            if self.debug:
                print(ff)
            self._dispatcher.send(ff)

            bytes_sent = 6
            sequence_number = 1

            #Force to wait for a flow control frame
            fc_bs = 1

            while bytes_sent < len(data):
                if fc_bs > 0:
                    fc_bs -= 1
                    if fc_bs == 0:
                        #Must wait for a flow control frame
                        start = time.time()
                        while True:
                            queu_timeout = (time.time() - start + N_Bs)
                            try:
                                rx_frame = self._recv_queue.get(timeout=queu_timeout)
                                if (rx_frame.arb_id == rx_arb_id and
                                        rx_frame.data[0] == 0x30):
                                    # flow control frame received
                                    fc_bs    = rx_frame.data[1]
                                    fc_stmin = rx_frame.data[2]
                                    break
                                if (time.time() - start) > N_Bs:
                                    raise Isotp_Error("N_Bs")
                            except Empty:
                                raise Isotp_Error("N_Bs")

                #Wait for fc_stmin ms/us
                if fc_stmin<0x80:
                    #fc_stmin equal to ms to wait
                    time_to_wait = fc_stmin/1000.0
                    #print ("Wait for " + str(time_to_wait))
                    time.sleep(time_to_wait)
                elif fc_stmin>=0xF1 and fc_stmin<=0xF9:
                    #fc_stmin equal to 100 - 900 us to wait
                    time_to_wait = (fc_stmin-0xF0)/1000000.0
                    #print ("Wait for " + str(time_to_wait))
                    time.sleep(time_to_wait)

                #Wait for N_Cs
                time.sleep(N_Cs)

                cf = can.Frame(tx_arb_id)
                data_bytes_in_msg = min(len(data) - bytes_sent, 7)

                frame_data = []
                frame_data.append(0x20 + sequence_number)
                frame_data = (frame_data +
                              data[bytes_sent:bytes_sent+data_bytes_in_msg])
                cf.data = self._pad_data(frame_data)

                if self.debug:
                    print(cf)
                self._dispatcher.send(cf)

                sequence_number = sequence_number + 1
                # wrap around when sequence number reaches 0xF
                if sequence_number > 0xF:
                    sequence_number = 0

                bytes_sent = bytes_sent + data_bytes_in_msg


class IsotpLinkLayer(IsotpInterface):
    def __init__(self, dispatcher, padding=0,debug=False):
        super(IsotpLinkLayer,self).__init__(dispatcher,padding,debug)
        self.N_As = 1.0
        self.N_Ar = 1.0
        self.N_Bs = 1.0
        self.N_Cr = 1.0
        self.N_Cs = 0.0
        
    def send_and_recv_physical(self, N_SA, N_TA, payload):
        self.send(payload,N_TA,N_SA,self.N_As,self.N_Bs,self.N_Cs)
        return self.recv(N_SA,N_TA)

    def send_Request(self, USDataRequest):
        try:
            self.send(USDataRequest.data,USDataRequest.N_TA,USDataRequest.N_SA,self.N_As,self.N_Bs,self.N_Cs)
            return IsotpN_USDataConfirm(USDataRequest)
        except Isotp_Error as e:
            return IsotpN_USDataConfirm(USDataRequest, e.Isotp_N_Result)

class IsotpNetworkLayer(IsotpLinkLayer):
    def __init__(self, dispatcher, padding=0,debug=False):
        super(IsotpNetworkLayer,self).__init__(dispatcher,padding,debug)




