#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "peer_localization.h"

#define N 10

typedef struct
{
    float buff_vx[N];
    float buff_vy[N];
    float buff_vz[N];
    uint32_t dt[N];
} peerVelocityBuffer_t;

void peerLocalizationInit()
{
  // All other_positions[in].id will be set to zero due to static initialization.
  // If we ever switch to dynamic allocation, we need to set them to zero explicitly.
}

bool peerLocalizationTest()
{
  return true;
}

// array of other's position
static peerLocalizationOtherPosition_t other_positions[PEER_LOCALIZATION_MAX_NEIGHBORS];
static peerLocalizationOtherState_t other_state[PEER_LOCALIZATION_MAX_NEIGHBORS];
static peerVelocityBuffer_t velocity_estimation_buffers[PEER_LOCALIZATION_MAX_NEIGHBORS];
static uint8_t vel_buff_idx[PEER_LOCALIZATION_MAX_NEIGHBORS];

void peerLocalizationFilterVelocity(uint8_t idx)
{
   // also handles ring buffer address underflow
   uint8_t b_idx_ctr = (vel_buff_idx[idx]-2)%N;
   uint8_t b_idx_l   = (vel_buff_idx[idx]-3)%N;
   uint8_t b_idx_ll  = (vel_buff_idx[idx]-4)%N;
   uint8_t b_idx_r   = (vel_buff_idx[idx]-1)%N;
   uint8_t b_idx_rr  = (vel_buff_idx[idx]);

   float * vx = velocity_estimation_buffers[idx].buff_vx;
   float * vy = velocity_estimation_buffers[idx].buff_vy;
   float * vz = velocity_estimation_buffers[idx].buff_vz;

   other_state[idx].id = other_positions[idx].id;
   other_state[idx].pos = other_positions[idx].pos;

   // Smoothing filter of window length 5. Current velocity state is two samples delayed
   other_state[idx].vel.x = 1/35*(-3*vx[b_idx_ll] + 12*vx[b_idx_l] + 17*vx[b_idx_ctr] + 12*vx[b_idx_r] + -3*vx[b_idx_rr]);
   other_state[idx].vel.y = 1/35*(-3*vy[b_idx_ll] + 12*vy[b_idx_l] + 17*vy[b_idx_ctr] + 12*vy[b_idx_r] + -3*vy[b_idx_rr]);
   other_state[idx].vel.z = 1/35*(-3*vz[b_idx_ll] + 12*vz[b_idx_l] + 17*vz[b_idx_ctr] + 12*vz[b_idx_r] + -3*vz[b_idx_rr]);

}


bool peerLocalizationTellPosition(int cfid, positionMeasurement_t const *pos)
{
  for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    if (other_positions[i].id == 0 || other_positions[i].id == cfid) {
      TickType_t curr_tick = xTaskGetTickCount();
      uint32_t dt = (curr_tick-other_positions[i].pos.timestamp)*portTICK_PERIOD_MS;
      velocity_estimation_buffers[i].buff_vx[vel_buff_idx[i]] = fabs(other_positions[i].pos.x - pos->x)/dt;
      velocity_estimation_buffers[i].buff_vy[vel_buff_idx[i]] = fabs(other_positions[i].pos.y - pos->y)/dt;
      velocity_estimation_buffers[i].buff_vz[vel_buff_idx[i]] = fabs(other_positions[i].pos.z - pos->z)/dt;
      velocity_estimation_buffers[i].dt[vel_buff_idx[i]] = dt;

      other_positions[i].id = cfid;
      other_positions[i].pos.x = pos->x;
      other_positions[i].pos.y = pos->y;
      other_positions[i].pos.z = pos->z;
      other_positions[i].pos.timestamp = curr_tick;

      vel_buff_idx[i]++;
      vel_buff_idx[i] = vel_buff_idx[i] % N;

      peerLocalizationFilterVelocity(i);

      return true;
    }
  }
  return false;
}

bool peerLocalizationIsIDActive(uint8_t cfid)
{
  for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    if (other_positions[i].id == cfid) {
      return true;
    }
  }
  return false;
}

peerLocalizationOtherPosition_t *peerLocalizationGetPositionByID(uint8_t cfid)
{
  for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    if (other_positions[i].id == cfid) {
      return &other_positions[i];
    }
  }
  return NULL;
}

peerLocalizationOtherState_t *peerLocalizationGetStateByID(uint8_t cfid)
{
    for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
        if (other_state[i].id == cfid) {
            return &other_state[i];
        }
    }
    return NULL;
}

peerLocalizationOtherPosition_t *peerLocalizationGetPositionByIdx(uint8_t idx)
{
  // TODO: should we return NULL if the id == 0?
  if (idx < PEER_LOCALIZATION_MAX_NEIGHBORS) {
    return &other_positions[idx];
  }
  return NULL;
}

peerLocalizationOtherState_t *peerLocalizationGetStateByIdx(uint8_t idx)
{
    if (idx < PEER_LOCALIZATION_MAX_NEIGHBORS) {
        return &other_state[idx];
    }
    return NULL;
}
