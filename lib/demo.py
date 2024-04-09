import fi_rcs

from fi_rcs.fi_flag_state import FlagState
from fi_rcs.fi_function_result import FunctionResult

from fi_rcs.fi_fsa_predefine import (FSAFunctionResult,
                                     FSAControlWord,
                                     FSAErrorCode,
                                     FSAModeOfOperation,
                                     FSAFlagState)
import fi_rcs.fi_fsa as fi_fsa

if __name__ == "__main__":
    print(FlagState.SET)
    print(FunctionResult.SUCCESS)

    # FSA
    fsa_ip = "192.168.137.100"

    # 1. init()
    fi_fsa.init(server_ip=fsa_ip)

    # 2. comm()
    fi_fsa.comm(server_ip=fsa_ip, enable=True, block=True)

    # 3. enable()
    fi_fsa.set_enable(server_ip=fsa_ip)

    # 4. set mode of operation
    fi_fsa.set_mode_of_operation(server_ip=fsa_ip, mode_of_operation=FSAModeOfOperation.POSITION_CONTROL)

    # 5. set position control
    fi_fsa.set_position_control(server_ip=fsa_ip, position=0, velocity_ff=0, current_ff=0)

    # 5. disable()
    fi_fsa.set_disable(server_ip=fsa_ip)
    