/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ins/ins_int.c
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#include "subsystems/ins/ins_int.h"

#include "subsystems/abi.h"

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"

#if USE_VFF_EXTENDED
#include "subsystems/ins/vf_extended_float.h"
#else
#include "subsystems/ins/vf_float.h"
#endif

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#if defined SITL && USE_NPS
//#include "nps_fdm.h"
#include "nps_autopilot.h"
#include <stdio.h>
#endif

#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"

#include "generated/flight_plan.h"


#if USE_SONAR
#if !USE_VFF_EXTENDED
#error USE_SONAR needs USE_VFF_EXTENDED
#endif

/** default sonar to use in INS */
#ifndef INS_SONAR_ID
#define INS_SONAR_ID ABI_BROADCAST
#endif
abi_event sonar_ev;
static void sonar_cb(uint8_t sender_id, float distance);

#ifdef INS_SONAR_THROTTLE_THRESHOLD
#include "firmwares/rotorcraft/stabilization.h"
#endif

#ifndef INS_SONAR_OFFSET
#define INS_SONAR_OFFSET 0.
#endif
#ifndef INS_SONAR_MIN_RANGE
#define INS_SONAR_MIN_RANGE 0.001
#endif
#define VFF_R_SONAR_0 0.1
#ifndef VFF_R_SONAR_OF_M
#define VFF_R_SONAR_OF_M 0.2
#endif

#ifndef INS_SONAR_UPDATE_ON_AGL
#define INS_SONAR_UPDATE_ON_AGL FALSE
PRINT_CONFIG_MSG("INS_SONAR_UPDATE_ON_AGL defaulting to FALSE")
#endif

#endif // USE_SONAR

#ifndef INS_VFF_R_GPS
#define INS_VFF_R_GPS 2.0
#endif

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

#ifdef INS_BARO_SENS
#warning INS_BARO_SENS is obsolete, please remove it from your airframe file.
#endif

/** default barometer to use in INS */
#ifndef INS_BARO_ID
#if USE_BARO_BOARD
#define INS_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_BARO_ID)
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, float pressure);

/** ABI binding for IMU data.
 * Used accel ABI messages.
 */
#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif
static abi_event accel_ev;
static abi_event gps_ev;

struct InsInt ins_int;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_int.baro_z, &ins_int.ltp_pos.z, &ins_int.ltp_speed.z, &ins_int.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_int.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_int.ltp_def.ecef.x, &ins_int.ltp_def.ecef.y, &ins_int.ltp_def.ecef.z,
                          &ins_int.ltp_def.lla.lat, &ins_int.ltp_def.lla.lon, &ins_int.ltp_def.lla.alt,
                          &ins_int.ltp_def.hmsl, &ins_int.qfe);
  }
}
#endif

static void ins_init_origin_from_flightplan(void);
static void ins_ned_to_state(void);
static void ins_update_from_vff(void);
#if USE_HFF
static void ins_update_from_hff(void);
#endif


void ins_int_init(void)
{

#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_int.ltp_initialized = TRUE;
#else
  ins_int.ltp_initialized  = FALSE;
#endif

  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
  ins_int.baro_initialized = FALSE;

#if USE_SONAR
  ins_int.update_on_agl = INS_SONAR_UPDATE_ON_AGL;
  // Bind to AGL message
  AbiBindMsgAGL(INS_SONAR_ID, &sonar_ev, sonar_cb);
#endif

  ins_int.vf_reset = FALSE;
  ins_int.hf_realign = FALSE;

  /* init vertical and horizontal filters */
  vff_init_zero();
#if USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif

  INT32_VECT3_ZERO(ins_int.ltp_pos);
  INT32_VECT3_ZERO(ins_int.ltp_speed);
  INT32_VECT3_ZERO(ins_int.ltp_accel);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "INS", send_ins);
  register_periodic_telemetry(DefaultPeriodic, "INS_Z", send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, "INS_REF", send_ins_ref);
#endif
}

void ins_reset_local_origin(void)
{
#if USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    ltp_def_from_ecef_i(&ins_int.ltp_def, &gps.ecef_pos);
    ins_int.ltp_def.lla.alt = gps.lla_pos.alt;
    ins_int.ltp_def.hmsl = gps.hmsl;
    ins_int.ltp_initialized = TRUE;
    stateSetLocalOrigin_i(&ins_int.ltp_def);
  }
  else {
    ins_int.ltp_initialized = FALSE;
  }
#else
  ins_int.ltp_initialized = FALSE;
#endif

#if USE_HFF
  ins_int.hf_realign = TRUE;
#endif
  ins_int.vf_reset = TRUE;
}

void ins_reset_altitude_ref(void)
{
#if USE_GPS
  struct LlaCoor_i lla = {
    .lat = state.ned_origin_i.lla.lat,
    .lon = state.ned_origin_i.lla.lon,
    .alt = gps.lla_pos.alt
  };
  ltp_def_from_lla_i(&ins_int.ltp_def, &lla);
  ins_int.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_int.ltp_def);
#endif
  ins_int.vf_reset = TRUE;
}

void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  struct Int32Vect3 accel_meas_ltp;
  int32_rmat_transp_vmult(&accel_meas_ltp, stateGetNedToBodyRMat_i(), &accel_meas_body);

  float z_accel_meas_float = ACCEL_FLOAT_OF_BFP(accel_meas_ltp.z);
  if (ins_int.baro_initialized) {
    vff_propagate(z_accel_meas_float, dt);
    ins_update_from_vff();
  } else { // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity,
    // but vehicle not accelerating in ltp)
    ins_int.ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }

#if USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate();
  /* convert and copy result to ins_int */
  ins_update_from_hff();
#else
  ins_int.ltp_accel.x = accel_meas_ltp.x;
  ins_int.ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  ins_ned_to_state();
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  if (!ins_int.baro_initialized && pressure > 1e-7) {
    // wait for a first positive value
    ins_int.qfe = pressure;
    ins_int.baro_initialized = TRUE;
  }

  if (ins_int.baro_initialized) {
    if (ins_int.vf_reset) {
      ins_int.vf_reset = FALSE;
      ins_int.qfe = pressure;
      vff_realign(0.);
      ins_update_from_vff();
    } else {
      ins_int.baro_z = -pprz_isa_height_of_pressure(pressure, ins_int.qfe);
#if USE_VFF_EXTENDED
      vff_update_baro(ins_int.baro_z);
#else
      vff_update(ins_int.baro_z);
#endif
    }
    ins_ned_to_state();
  }
}

#if USE_GPS
void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix == GPS_FIX_3D) {
    if (!ins_int.ltp_initialized) {
      ltp_def_from_ecef_i(&ins_int.ltp_def, &gps_s->ecef_pos);
      ins_int.ltp_def.lla.alt = gps_s->lla_pos.alt;
      ins_int.ltp_def.hmsl = gps_s->hmsl;
      ins_int.ltp_initialized = TRUE;
      stateSetLocalOrigin_i(&ins_int.ltp_def);
    }

    struct NedCoor_i gps_pos_cm_ned;
    ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &gps_s->ecef_pos);

    /* calculate body frame position taking BODY_TO_GPS translation (in cm) into account */
#ifdef INS_BODY_TO_GPS_X
    /* body2gps translation in body frame */
    struct Int32Vect3 b2g_b = {
      .x = INS_BODY_TO_GPS_X,
      .y = INS_BODY_TO_GPS_Y,
      .z = INS_BODY_TO_GPS_Z
    };
    /* rotate offset given in body frame to navigation/ltp frame using current attitude */
    struct Int32Quat q_b2n;
    memcpy(&q_b2n, stateGetNedToBodyQuat_i(), sizeof(struct Int32Quat));
    QUAT_INVERT(q_b2n, q_b2n);
    struct Int32Vect3 b2g_n;
    int32_quat_vmult(&b2g_n, &q_b2n, &b2g_b);
    /* subtract body2gps translation in ltp from gps position */
    VECT3_SUB(gps_pos_cm_ned, b2g_n);
#endif

    /// @todo maybe use gps_s->ned_vel directly??
    struct NedCoor_i gps_speed_cm_s_ned;
    ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_int.ltp_def, &gps_s->ecef_vel);

#if INS_USE_GPS_ALT
    vff_update_z_conf((float)gps_pos_cm_ned.z / 100.0, INS_VFF_R_GPS);
#endif

#if USE_HFF
    /* horizontal gps transformed to NED in meters as float */
    struct FloatVect2 gps_pos_m_ned;
    VECT2_ASSIGN(gps_pos_m_ned, gps_pos_cm_ned.x, gps_pos_cm_ned.y);
    VECT2_SDIV(gps_pos_m_ned, gps_pos_m_ned, 100.0f);

    struct FloatVect2 gps_speed_m_s_ned;
    VECT2_ASSIGN(gps_speed_m_s_ned, gps_speed_cm_s_ned.x, gps_speed_cm_s_ned.y);
    VECT2_SDIV(gps_speed_m_s_ned, gps_speed_m_s_ned, 100.);

    if (ins_int.hf_realign) {
      ins_int.hf_realign = FALSE;
      const struct FloatVect2 zero = {0.0f, 0.0f};
      b2_hff_realign(gps_pos_m_ned, zero);
    }
    // run horizontal filter
    b2_hff_update_gps(&gps_pos_m_ned, &gps_speed_m_s_ned);
    // convert and copy result to ins_int
    ins_update_from_hff();

#else  /* hff not used */
    /* simply copy horizontal pos/speed from gps */
    INT32_VECT2_SCALE_2(ins_int.ltp_pos, gps_pos_cm_ned,
                        INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(ins_int.ltp_speed, gps_speed_cm_s_ned,
                        INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* USE_HFF */

    ins_ned_to_state();
  }
}
#else
void ins_int_update_gps(struct GpsState *gps_s __attribute__((unused))) {}
#endif /* USE_GPS */


#if USE_SONAR
static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  static float last_offset = 0.;

  /* update filter assuming a flat ground */
  if (distance < INS_SONAR_MAX_RANGE && distance > INS_SONAR_MIN_RANGE
#ifdef INS_SONAR_THROTTLE_THRESHOLD
      && stabilization_cmd[COMMAND_THRUST] < INS_SONAR_THROTTLE_THRESHOLD
#endif
#ifdef INS_SONAR_BARO_THRESHOLD
      && ins_int.baro_z > -INS_SONAR_BARO_THRESHOLD /* z down */
#endif
      && ins_int.update_on_agl
      && ins_int.baro_initialized) {
    vff_update_z_conf(-(distance), VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabsf(distance));
    last_offset = vff.offset;
  } else {
    /* update offset with last value to avoid divergence */
    vff_update_offset(last_offset);
  }
}
#endif // USE_SONAR


/** initialize the local origin (ltp_def) from flight plan position */
static void ins_init_origin_from_flightplan(void)
{

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_int.ltp_def, &ecef_nav0);
  ins_int.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_int.ltp_def);

}

/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(&ins_int.ltp_pos);
  stateSetSpeedNed_i(&ins_int.ltp_speed);
  stateSetAccelNed_i(&ins_int.ltp_accel);

#if defined SITL && USE_NPS
  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }
#endif
}

/** update ins state from vertical filter */
static void ins_update_from_vff(void)
{
  ins_int.ltp_accel.z = ACCEL_BFP_OF_REAL(vff.zdotdot);
  ins_int.ltp_speed.z = SPEED_BFP_OF_REAL(vff.zdot);
  ins_int.ltp_pos.z   = POS_BFP_OF_REAL(vff.z);
}

#if USE_HFF
/** update ins state from horizontal filter */
static void ins_update_from_hff(void)
{
  ins_int.ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
  ins_int.ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
  ins_int.ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
  ins_int.ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
  ins_int.ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
  ins_int.ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);
}
#endif


static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  PRINT_CONFIG_MSG("Calculating dt for INS int propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_int_propagate(accel, dt);
  }
  last_stamp = stamp;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_int_update_gps(gps_s);
}

void ins_int_register(void)
{
  ins_register_impl(ins_int_init);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_ACCEL_INT32(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}
