READY_HLC_STATE = 4
STANDBY_MC_STATE = 2


def system_ready(world_model) -> bool:
    return world_model.hlc_state == READY_HLC_STATE and world_model.mc_state == STANDBY_MC_STATE
