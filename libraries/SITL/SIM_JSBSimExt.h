/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connection for JSBSim - https://github.com/JSBSim-Team/jsbsim
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_JSBSIM_EXT_ENABLED
#define HAL_SIM_JSBSIM_EXT_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_JSBSIM_EXT_ENABLED

#include <AP_HAL/utility/Socket_native.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a Jsbsim simulator
 */
class JSBSimExt : public Aircraft {
public:
    JSBSimExt(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new JSBSimExt(frame_str);
    }

private:
    // tcp input control socket to JSBSIm
    SocketAPM_native sock_control;

    // UDP packets from JSBSim in fgFDM format
    SocketAPM_native sock_fgfdm;

    bool initialised;

    uint16_t control_port;
    uint16_t fdm_port;

    // default JSBSim model
    const char *jsbsim_model = "Rascal";

    bool created_templates;
    bool opened_control_socket;
    bool opened_fdm_socket;

    enum {
        FRAME_NORMAL,
        FRAME_ELEVON,
        FRAME_VTAIL
    } frame;

    bool create_templates(void);
    bool open_control_socket(void);
    bool open_fdm_socket(void);
    void send_servos(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);

    void drain_control_socket();
};

} // namespace SITL

#endif  // HAL_SIM_JSBSIM_ENABLED
