class VROBOTS_CMDS:
    # linear motion
    SET_ACC = 1
    SET_VEL = 2
    SET_POS = 3

    # angular motion
    SET_ANGACC = 50
    SET_ANGVEL = 51
    SET_EULER = 52
    SET_EULER_DOT = 53
    SET_QUAT = 54

    # mass
    SET_MASS = 100
    SET_MOI_3X1 = 101
    SET_MOI_3X3 = 102

    # set body forces and torques
    SET_BODY_FORCE = 200
    SET_BODY_TORQUE = 201
    SET_BODY_FT = 202
    ADD_BODY_FORCE = 203
    ADD_BODY_TORQUE = 204
    ADD_BODY_FT = 205

    # set acutators
    SET_PWM = 300
    SET_MR_THROTTLE = 301
    SET_OMROVER = 302
    SET_HELI = 303
    SET_CAR = 304
