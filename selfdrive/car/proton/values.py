# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CAR:
  PROTON_SAGA = "PROTON SAGA"
  PROTON_IRIZ = "PROTON IRIZ"
  PROTON_PERSONA = "PROTON PERSONA"

FINGERPRINTS = {
  CAR.PROTON_SAGA: [{
    257: 8, 277: 8, 512: 8, 513: 8, 528: 8, 529: 8, 625: 8, 626: 8, 627: 8, 657: 8, 776: 8, 784: 8, 786: 8, 864: 8, 961: 8, 1296: 8, 1776: 8, 1791: 3, 1827: 8, 1985: 8
  }],
  CAR.PROTON_IRIZ: [{
    257: 8, 277: 8, 419: 8, 512: 8, 513: 8, 528: 8, 529: 8, 625: 8, 626: 8, 627: 8, 657: 8, 753: 3, 776: 8, 784: 8, 786: 8, 836: 5, 848: 8, 961: 8, 1026: 5, 1042: 5, 1058: 5, 1074: 5, 1156: 3, 1160: 8, 1236: 3, 1245: 8, 1247: 8, 1263: 8, 1279: 8, 1296: 8, 1542: 3, 1555: 8, 1558: 8, 1776: 8, 1791: 3, 1985: 8
  }],
#  CAR.PROTON_PERSONA: [{
#
#  }],
}

DBC = {
  CAR.PROTON_SAGA: dbc_dict('proton_general_pt',None),
  CAR.PROTON_IRIZ: dbc_dict('proton_general_pt',None),
#  CAR.PROTON_PERSONA: dbc_dict('proton_general_pt',None)

}

NOT_CAN_CONTROLLED = set([CAR.PROTON_SAGA, CAR.PROTON_IRIZ])
