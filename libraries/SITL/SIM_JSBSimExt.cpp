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
  simulator connector for JSBSim
*/

#include "SIM_JSBSimExt.h"

#if HAL_SIM_JSBSIM_EXT_ENABLED

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>
// We use the FGNetFDM class defined in SIM_JSBSim.h
#include <SITL/SIM_JSBSim.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM_EXT 1

JSBSimExt::JSBSimExt(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    initialised(false),
    created_templates(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL)
{
    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else {
        frame = FRAME_NORMAL;
    }
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        jsbsim_model = model_name + 1;
    }
    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;

    printf("JSBSim backend started: control_port=%u fdm_port=%u\n",
           control_port, fdm_port);
}

/*
  create template files
 */
bool JSBSimExt::create_templates(void)
{
    if (created_templates) {
        return true;
    }

    char *jsbsim_script;
    asprintf(&jsbsim_script, "%s/jsbsim_start_%u.xml", autotest_dir, instance);
    char *jsbsim_fgout;
    asprintf(&jsbsim_fgout,  "%s/jsbsim_fgout_%u.xml", autotest_dir, instance);

    printf("JSBSim_script: '%s'\n", jsbsim_script);
    printf("JSBSim_fgout: '%s'\n", jsbsim_fgout);

    FILE *f = fopen(jsbsim_script, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim script %s", jsbsim_script);
    }
    fprintf(f,
"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
"<?xml-stylesheet type=\"text/xsl\" href=\"http://jsbsim.sf.net/JSBSimScript.xsl\"?>\n"
"<runscript xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
"    xsi:noNamespaceSchemaLocation=\"http://jsbsim.sf.net/JSBSimScript.xsd\"\n"
"    name=\"Testing %s\">\n"
"\n"
"  <description>\n"
"    test ArduPlane using %s and JSBSim\n"
"  </description>\n"
"\n"
"  <use aircraft=\"%s\" initialize=\"reset\"/>\n"
"\n"
"  <!-- we control the servos via the jsbsim console\n"
"       interface on TCP 5124 -->\n"
"  <input port=\"%u\"/>\n"
"\n"
"  <run start=\"0\" end=\"10000000\" dt=\"%.6f\">\n"
"    <property value=\"0\"> simulation/notify-time-trigger </property>\n"
"\n"
"    <event name=\"start engine\">\n"
"      <condition> simulation/sim-time-sec le 0.01 </condition>\n"
"      <set name=\"propulsion/engine[0]/set-running\" value=\"1\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n"
"    <event name=\"Trim\">\n"
"      <condition>simulation/sim-time-sec ge 0.01</condition>\n"
"      <set name=\"simulation/do_simple_trim\" value=\"2\"/>\n"
"      <notify/>\n"
"    </event>\n"
"  </run>\n"
"\n"
"</runscript>\n"
"",
            jsbsim_model,
            jsbsim_model,
            jsbsim_model,
            // We still connect to `control_port`, but configure JSBSim to
            // listen on a different port instead.  `jsbsim_proxy` will forward
            // traffic between the two (after starting JSBSim).
            control_port + 10000,
            1.0/rate_hz);
    fclose(f);

    f = fopen(jsbsim_fgout, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim fgout script %s", jsbsim_fgout);
    }
    fprintf(f, "<?xml version=\"1.0\"?>\n"
            "<output name=\"127.0.0.1\" type=\"FLIGHTGEAR\" port=\"%u\" protocol=\"UDP\" rate=\"%f\">\n"
            "  <time type=\"simulation\" resolution=\"1e-6\"/>\n"
            "</output>",
            fdm_port, rate_hz);
    fclose(f);

    char *jsbsim_reset;
    asprintf(&jsbsim_reset, "%s/aircraft/%s/reset.xml", autotest_dir, jsbsim_model);

    printf("JSBSim_reset: '%s'\n", jsbsim_reset);

    f = fopen(jsbsim_reset, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim reset script %s", jsbsim_reset);
    }
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fprintf(f,
            "<?xml version=\"1.0\"?>\n"
            "<initialize name=\"Start up location\">\n"
            "  <latitude unit=\"DEG\" type=\"geodetic\"> %f </latitude>\n"
            "  <longitude unit=\"DEG\"> %f </longitude>\n"
            "  <altitude unit=\"M\"> 1.3 </altitude>\n"
            "  <vt unit=\"FT/SEC\"> 0.0 </vt>\n"
            "  <gamma unit=\"DEG\"> 0.0 </gamma>\n"
            "  <phi unit=\"DEG\"> 0.0 </phi>\n"
            "  <theta unit=\"DEG\"> 13.0 </theta>\n"
            "  <psi unit=\"DEG\"> %f </psi>\n"
            "</initialize>\n",
            home.lat*1.0e-7,
            home.lng*1.0e-7,
            degrees(y));
    fclose(f);

    created_templates = true;
    return true;
}

/*
  open control socket to JSBSim
 */
bool JSBSimExt::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    printf("JSBSim backend: connecting to %s:%u\n", control_address, control_port);
    if (!sock_control.connect(control_address, control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    char startup[] =
        "info\n"
        "resume\n"
        "iterate 1\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSimExt::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    printf("JSBSim backend: listening on %s:%u\n", fdm_bind_address, fdm_port);
    if (!sock_fgfdm.bind(fdm_bind_address, fdm_port)) {
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;
    return true;
}


/*
  decode and send servos
*/
void JSBSimExt::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    if (frame == FRAME_ELEVON) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;
    } else if (frame == FRAME_VTAIL) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    }
    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
    asprintf(&buf,
             "set fcs/aileron-cmd-norm %f\n"
             "set fcs/elevator-cmd-norm %f\n"
             "set fcs/rudder-cmd-norm %f\n"
             "set fcs/throttle-cmd-norm %f\n"
             "set atmosphere/psiw-rad %f\n"
             "set atmosphere/wind-mag-fps %f\n"
             "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
             "set atmosphere/turbulence/milspec/severity %f\n"
             "iterate 1\n",
             aileron, elevator, rudder, throttle,
             radians(input.wind.direction),
             wind_speed_fps,
             wind_speed_fps/3,
             input.wind.turbulence);
    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);
    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSimExt::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;

    do {
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 100) != sizeof(fdm)) {
            send_servos(input);
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    location.lat = RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7;
    location.lng = RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7;
    location.alt = fdm.agl*100 + home.alt;
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();
    
    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];
    
    time_now_us = fdm.cur_time;
}

void JSBSimExt::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
void JSBSimExt::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!create_templates() ||
            !open_fdm_socket() ||
            !open_control_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    send_servos(input);
    recv_fdm(input);
    adjust_frame_time(rate_hz);
    sync_frame_time();
    drain_control_socket();
}

void JSBSimExt::set_interface_ports(const char* address, const int port_in, const int port_out) {
    control_address = address;
    control_port = port_out;
    fdm_port = port_in;

    if (strcmp(address, "127.0.0.1") == 0) {
        fdm_bind_address = "127.0.0.1";
    } else {
        fdm_bind_address = "0.0.0.0";
    }
    printf("JSBSim backend: set control address to %s:%u\n", control_address, control_port);
    printf("JSBSim backend: set fdm bind address to %s:%u\n", fdm_bind_address, fdm_port);
}

} // namespace SITL

#endif  // HAL_SIM_JSBSIM_ENABLED
