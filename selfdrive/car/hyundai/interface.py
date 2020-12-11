#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import CAR, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from common.dp_common import common_interface_atl, common_interface_get_params_lqr
from common.params import Params

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.buttonEvents = []
    self.cp2 = self.CS.get_can2_parser(CP)
    self.visiononlyWarning = False
    self.belowspeeddingtimer = 0.
    self.enabled_prev = False
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai


    # Most Hyundai car ports are community features for now
    ret.communityFeature = candidate not in [CAR.SONATA, CAR.PALISADE]

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 1.
    #Long tuning Params -  make individual params for cars, baseline Hyundai genesis    
    ret.longitudinalTuning.kpBP = [0., .3, 10., 35.]    
    ret.longitudinalTuning.kpV = [1.8, .8, .3, .3]    
    ret.longitudinalTuning.kiBP = [0., .3, 15., 35.]    
    ret.longitudinalTuning.kiV = [0.15, .08, .06, .05]    
    ret.longitudinalTuning.deadzoneBP = [0., .5]    
    ret.longitudinalTuning.deadzoneV = [0.00, 0.00]    
    ret.gasMaxBP = [0., 1., 1.1, 15., 40.]    
    ret.gasMaxV = [2., 2., 2., 1.68, 1.3]    
    ret.brakeMaxBP = [0., 5., 5.1]    
    ret.brakeMaxV = [3.5, 3.5, 3.5]  
    # safety limits to stop unintended deceleration    
    ret.longitudinalTuning.kfBP = [0., 5., 10., 20., 30.]    
    ret.longitudinalTuning.kfV = [1., 1., 1., .75, .5]
    
    ret.lateralTuning.pid.kpBP = [0., 10., 30.]
    ret.lateralTuning.pid.kpV = [0.01, 0.02, 0.03]
    ret.lateralTuning.pid.kiBP = [0., 10., 30.]
    ret.lateralTuning.pid.kiV = [0.001, 0.0015, 0.002]
    ret.lateralTuning.pid.kfBP = [0., 10., 30.]
    ret.lateralTuning.pid.kfV = [0.000015, 0.00002, 0.000025]
    
       
    if Params().get('Enable_INDI'):
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.outerLoopGain = 3.  # stock is 2.0.  Trying out 2.5
      ret.lateralTuning.indi.innerLoopGain = 2.
      ret.lateralTuning.indi.timeConstant = 1.4
      ret.lateralTuning.indi.actuatorEffectiveness = 2.

    if candidate in [CAR.SANTA_FE]:
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82

    elif candidate == CAR.SONATA:
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
    elif candidate == CAR.SONATA_2019:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable

    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 13.75 * 1.15

    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:

      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      tire_stiffness_factor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 15
      # dp - indi value from donfyffe
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGain = 3.1
      ret.lateralTuning.indi.outerLoopGain = 2.1
      ret.lateralTuning.indi.timeConstant = 1.4
      ret.lateralTuning.indi.actuatorEffectiveness = 1.4
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
      ret.minEnableSpeed = 10 * CV.MPH_TO_MS
    elif candidate == CAR.KONA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15  # Spec
      tire_stiffness_factor = 0.385
    elif candidate == CAR.KONA_EV:
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
    elif candidate in [CAR.IONIQ, CAR.IONIQ_EV_LTD]:
      ret.mass = 1490. + STD_CARGO_KG   #weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73   #Spec
      tire_stiffness_factor = 0.385
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.VELOSTER:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      tire_stiffness_factor = 0.5

    # Kia
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate == CAR.KIA_NIRO_EV:
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
    elif candidate in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_H]:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_STINGER:
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5

    # Genesis
    elif candidate == CAR.GENESIS_G70:
      ret.lateralTuning.init('indi') # TODO: BPs for city speeds - this tuning is great on the highway but a bit lazy in town
      ret.lateralTuning.indi.innerLoopGain = 2.4  # higher values steer more
      ret.lateralTuning.indi.outerLoopGain = 3.0  # higher values steer more
      ret.lateralTuning.indi.timeConstant = 1.0  # lower values steer more
      ret.lateralTuning.indi.actuatorEffectiveness = 2.0  # lower values steer more
      ret.steerActuatorDelay = 0.4 # 0.08 stock
      ret.steerLimitTimer = 0.4 # down from 0.4
      tire_stiffness_factor = 1.0
      ret.steerRateCost = 1.0
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.906
      ret.steerRatio = 14.4
    elif candidate == CAR.GENESIS_G80:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGain = 3.5
      ret.lateralTuning.indi.outerLoopGain = 2.0
      ret.lateralTuning.indi.timeConstant = 1.4
      ret.lateralTuning.indi.actuatorEffectiveness = 2.3
      ret.minSteerSpeed = 60 * CV.KPH_TO_MS
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069

    # these cars require a special panda safety mode due to missing counters and checksums in the messages
    
    ret.mdpsHarness = Params().get('MdpsHarnessEnabled') == b'1'    
    ret.sasBus = 0 if (688 in fingerprint[0] or not ret.mdpsHarness) else 1    
    ret.fcaBus = 0 if 909 in fingerprint[0] else 2 if 909 in fingerprint[2] else -1    
    ret.bsmAvailable = True if 1419 in fingerprint[0] else False    
    ret.lfaAvailable = True if 1157 in fingerprint[2] else False    
    ret.lvrAvailable = True if 871 in fingerprint[0] else False    
    ret.evgearAvailable = True if 882 in fingerprint[0] else False    
    ret.emsAvailable = True if 608 and 809 in fingerprint[0] else False      
    ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy
    
    if candidate in [CAR.HYUNDAI_GENESIS, CAR.IONIQ_EV_LTD, CAR.IONIQ, CAR.KONA_EV, CAR.KIA_SORENTO, CAR.SONATA_2019,
                     CAR.KIA_NIRO_EV, CAR.KIA_OPTIMA, CAR.VELOSTER, CAR.KIA_STINGER, CAR.GENESIS_G70, CAR.GENESIS_G80]:
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy

      
    if Params().get('SccEnabled') == b'1':
      ret.sccBus = 2 if 1057 in fingerprint[2] and Params().get('SccHarnessPresent') == b'1' else 0 if 1057 in fingerprint[0] else -1
    else:
      ret.sccBus = -1
    
    
    ret.radarOffCan = (ret.sccBus == -1)
    
    ret.openpilotLongitudinalControl = Params().get('LongControlEnabled') == b'1' and not (ret.sccBus == 0)
    if ret.openpilotLongitudinalControl:
      ret.radarTimeStep = .05
  
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy
    
    if ret.mdpsHarness or \
            (candidate in [CAR.KIA_OPTIMA_HEV, CAR.SONATA_HEV, CAR.IONIQ_HEV, CAR.SONATA_HEV_2019,
                          CAR.KIA_CADENZA_HEV, CAR.GRANDEUR_HEV, CAR.KIA_NIRO_HEV, CAR.KONA_HEV]):
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunity
    
    if ret.radarOffCan or (ret.sccBus == 2) or Params().get('EnableOPwithCC') == b'0':
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunityNonscc

    if ret.mdpsHarness or Params().get('smartMDPS'):
      ret.minSteerSpeed = 0.
    
    
    
    
    ret.centerToFront = ret.wheelbase * 0.4

    # dp
    if Params().get('dp_hkg_smart_mdps') == b'1':
        ret.minSteerSpeed = 0.
    ret = common_interface_get_params_lqr(ret)
     

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = True

    ret.radarDisablePossible = Params().get('RadarDisableEnabled') == b'1'

    ret.enableCruise = Params().get('EnableOPwithCC') == b'1' and ret.sccBus == 0

    if ret.radarDisablePossible:
      ret.openpilotLongitudinalControl = True
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunityNonscc # todo based on toggle
      ret.sccBus = -1
      ret.enableCruise = False
      ret.radarOffCan = True
      if ret.fcaBus == 0:
        ret.fcaBus = -1
    return ret

  def update(self, c, can_strings, dragonconf):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid


    # dp
    self.dragonconf = dragonconf
    if ret.vEgo >= self.CP.minSteerSpeed:
      ret.cruiseState.enabled = common_interface_atl(ret, dragonconf.dpAtl)


    events = self.create_common_events(ret)
    
    #TODO: addd abs(self.CS.angle_steers) > 90 to 'steerTempUnavailable' event



    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + .56) and self.CP.minSteerSpeed > 10. and self.CC.enabled:
      if not self.low_speed_alert and self.belowspeeddingtimer < 100:
        events.add(car.CarEvent.EventName.belowSteerSpeedDing)
        self.belowspeeddingtimer +=1
      else:
        self.belowspeeddingtimer = 0.
        self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + .84) or not self.CC.enabled:
      self.low_speed_alert = False
      self.belowspeeddingtimer = 0.
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    if self.CP.sccBus == 2:
      self.CP.enableCruise = self.CC.usestockscc

    if self.enabled_prev and not self.CC.enabled and not self.CP.enableCruise:
      ret.cruiseState.enabled = False
    self.enabled_prev = self.CC.enabled

    if self.CS.brakeHold and not self.CC.usestockscc:
      events.add(EventName.brakeHold)
    if self.CS.parkBrake and not self.CC.usestockscc:
      events.add(EventName.parkBrake)
    if self.CS.brakeUnavailable and not self.CC.usestockscc:
      events.add(EventName.brakeUnavailable)
    if not self.visiononlyWarning and self.CP.radarDisablePossible and self.CC.enabled and not self.low_speed_alert:
      events.add(EventName.visiononlyWarning)
      self.visiononlyWarning = True

    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = self.CS.cruise_buttons != 0 
      but = self.CS.cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      else:
        be.type = ButtonType.unknown
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    ret.buttonEvents = self.buttonEvents

    # handle button press
    if not self.CP.enableCruise:
      for b in self.buttonEvents:
        if b.type == ButtonType.decelCruise and b.pressed \
                and (not ret.brakePressed or ret.standstill):
          events.add(EventName.buttonEnable)
          events.add(EventName.pcmEnable)
        if b.type == ButtonType.accelCruise and b.pressed \
                and ((self.CC.setspeed > self.CC.clu11_speed - 2) or ret.standstill or self.CC.usestockscc):
          events.add(EventName.buttonEnable)
          events.add(EventName.pcmEnable)
        if b.type == ButtonType.cancel and b.pressed or self.CS.lkasbutton and opParams().get('enableLKASbutton'):
          events.add(EventName.buttonCancel)
          events.add(EventName.pcmDisable)
        if b.type == ButtonType.altButton3 and b.pressed:
          events.add(EventName.buttonCancel)
          events.add(EventName.pcmDisable)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel,
                               c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                               c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)
    self.frame += 1
    return can_sends
