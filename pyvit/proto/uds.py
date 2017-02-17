from pyvit.proto.isotp import *
import time


class UdsInterface(IsotpNetworkLayer):
    def __init__(self, dispatcher, padding=0,debug=False):
        super(UdsInterface,self).__init__(dispatcher,padding,debug)

    def _request(self,req):
        ret = self.send_Request(req)
        if(ret.N_Result != Isotp_N_Result.N_OK):
            return [False ,"network",ret.N_Result]
        ret = self.wait_IndicationForConfirm(ret)
        if(ret.N_Result != Isotp_N_Result.N_OK):
            return [False ,"network",ret.N_Result]
        if(ret.data[0] == 127):
           return [False, "uds", "service not supported"]
        return [True, ret.data]

    def _createRequest(self,ecuId,payload):
        return IsotpN_USDataRequest(Isotp_Mtype.diagnostic, ecuId+8, ecuId, Isoyp_N_TAtype.physical, payload)

    def requestSession(self, ecuId, sessionId):
        payload = [0x10, sessionId]
        req = self._createRequest(ecuId, payload)
        return self._request(req)

    def requestInputOutputControlByIdentifier(self, ecuId, identifier, value):
        idLsb = identifier & 0xFF
        ifMsb = (identifier >> 8) & 0xFF
        payload = [0x2F, ifMsb, idLsb, 0x03, value]
        req = self._createRequest(ecuId, payload)
        return self._request(req)
    

