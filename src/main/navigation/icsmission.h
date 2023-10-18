#ifndef _APMISSION_H_
#define _APMISSION_H_

#include <stdint.h>
#include "types.h"


//typedef struct gtmissionpoint_tag {
//    uint32_t la;
//    uint32_t lo;
//    uint16_t h;
//    uint8_t valid;
//} GTMissionPoint;

typedef struct xfloatpoint_tag {
    float x;
    float y;
} XFloatPoint;


typedef struct xmissionpoint_tag {
    int32_t la;
    int32_t lo;
    int32_t h;
    int alt_h;
    uint8_t kind;
    uint8_t p1;
    float x;
    float y;
    uint16_t pdist;
    uint8_t object_id;
    uint8_t object_type;

    // я добавил это только чтобы было у точки дом
    float inavX;
    float inavY;
} XMissionPoint;

extern XMissionPoint sl, el;

#define GT_MISSION_CAPACITY (256)

#define XMISSION_KIND_POINT (0)
#define XMISSION_KIND_LINE_START (1)
#define XMISSION_KIND_LINE_END (2)
#define XMISSION_KIND_PRELANDING (4)
#define XMISSION_KIND_LAND (3)
#define XMISSION_KIND_COPTER (5)

#define XMISSION_KIND_CIRCLE_INDEF (10)


extern uint16_t XM_Count;
extern uint16_t XM_Current;
extern uint8_t is_mission_valid;
extern uint8_t is_home_valid;

uint8_t XM_AddReceived( uint16_t cur, uint16_t size, int32_t la, int32_t lo, int32_t h, uint8_t kind, uint8_t p1, uint16_t pdist, uint8_t object_id, uint8_t object_type  );

XMissionPoint XM_GetCurrent();
XMissionPoint XM_GetNext();
void XM_Prev();
void XM_Next();
XMissionPoint XM_GetHome();
XMissionPoint XM_GetPrelanding();
XMissionPoint XM_Get(int ix);
uint8_t XM_SetCurrent( uint16_t new_point_number);
void XM_Reached();
uint8_t XM_IsPenult();
uint32_t X_Dist2_m();
uint8_t XM_IsLine();
uint8_t XM_IsCopter();
uint8_t XM_IsCircle();

void set_XM_Current (uint16_t  newNumber);

XMissionPoint XM_GetLineStart();
XFloatPoint XM_ToLocal( int32_t la, int32_t lo);

uint32_t x_dist_between_points_m(uint16_t cur_point1, uint16_t cur_point2 );
void XM_reset_local_systems(void);

//extern GTMissionPoint gtmission[GT_MISSION_CAPACITY];
extern uint8_t gtmission_count;
extern uint8_t cur_point;

#endif
