#pragma once

/**
 * @brief VTD RDB parse msg
 * @param perception_obstacles
 * @param lane_info
 * @param road_mark
 * @param traffic_sign
 * @param traffic_light
 * @param sensor_object
 */


/** ------ state of an object ------- */
typedef struct {
    double      id;                      /**< obstacles id                                                                 @unit _                              */
    double      category;                /**< obstacles category                                                           @unit _                              */
    double      type;                    /**< obstacles type[vehicle pedestrians bycle]                                    @unit _                              */
    double      visMask;
    double      name;
    double      pos_x;                   /**< position x                                                                   @unit m                              */
    double      pos_y;                   /**< position y                                                                   @unit m                              */
    double      pos_z;                   /**< position z                                                                   @unit m                              */
    double      pos_h;                   /**< heading                                                                      @unit rad                            */
    double      pos_p;                   /**< pitch                                                                        @unit rad                            */
    double      pos_r;                   /**< roll                                                                         @unit rad                            */
    double      parent;
    double      cfgFlags;
    double      cfgModelId;
    double      speed_x;                 /**< velocity x                                                                   @unit m/s                            */
    double      speed_y;                 /**< velocity y                                                                   @unit m/s                            */
    double      speed_z;                 /**< velocity z                                                                   @unit m/s                            */
    double      accel_x;                 /**< accleration x                                                                @unit m/s2                           */
    double      accel_y;                 /**< accleration y                                                                @unit m/s2                           */
    double      accel_z;                 /**< accleration z                                                                @unit m/s2                           */
    double      length;                  /**< bounding box length                                                          @unit m                              */
    double      width;                   /**< bounding box width                                                           @unit m                              */
    double      height;                  /**< bounding box height                                                          @unit m                              */
} RDB_OBJECT_STATE;

/** ------ lane information ------ */
typedef struct
{
    double      roadId;                  /**< unique road ID                                                               @unit _                              */
    double      id;                      /**< lane ID according to OpenDRIVE                                               @unit [-127..127]                    */
    double      neighborMask;            /**< existence mask for adjacent lanes                                            @unit                                */
    double      leftLaneId;              /**< ID of lane left of current lane                                              @unit [-127..127]                    */
    double      rightLaneId;             /**< ID of lane right of current lane                                             @unit [-127..127]                    */
    double      borderType;              /**< type of lane border                                                          @unit                                */
    double      material;                /**< type of lane material                                                        @unit [0..255]                       */
    double      status;                  /**< status mask of lane                                                          @unit                                */
    double      type;                    /**< enumerated lane type according to OpenDRIVE (0=none, 1=driving...)           @unit _                              */
    double      width;                   /**< lane width                                                                   @unit m                              */
    double      curvVert;                /**< vertical curvature in lane center                                            @unit 1/m                            */
    double      curvVertDot;             /**< change of vertical curvature in lane center                                  @unit 1/m2                           */
    double      curvHor;                 /**< horizontal curvature in lane center                                          @unit 1/m                            */
    double      curvHorDot;              /**< change of horizontal curvature in lane center                                @unit 1/m2                           */
    double      playerId;                /**< id of the player to which this info belongs                                  @unit _                              */
    double      spare1;                  /**< for future use                                                               @unit _                              */
} RDB_LANE_INFO;

/** ------ road mark information ------
 * @note this package is immediately followed by "noDataPoints" entries of type RDB_POINT_t
 */
typedef struct
{
    double      playerId;                /**< id of the player to which roadmark belongs                                   @unit _                               */
    double      id;                      /**< id of this road mark                                                         @unit [0..127]                        */
    double      prevId;                  /**< id of predecessor                                                            @unit [-1, 0..127]                    */
    double      nextId;                  /**< id of successor                                                              @unit [-1, 0..127]                    */
    double      laneId;                  /**< id of the lane to which the roadmark belongs                                 @unit _                               */
    double      lateralDist;             /**< lateral distance to vehicle ref. point and dir                               @unit m                               */
    double      yawRel;                  /**< yaw angle relative to vehicle dir                                            @unit rad [-PI;PI]                    */
    double      curvHor;                 /**< horizontal curvature                                                         @unit 1/m                             */
    double      curvHorDot;              /**< change of horizontal curvature                                               @unit 1/m2                            */
    double      startDx;                 /**< start of road mark in driving dir                                            @unit m                               */
    double      previewDx;               /**< distance of last valid measurement                                           @unit m                               */
    double      width;                   /**< width of road mark                                                           @unit m                               */
    double      height;                  /**< height of road mark                                                          @unit m                               */
    double      curvVert;                /**< vertical curvature                                                           @unit 1/m                             */
    double      curvVertDot;             /**< change of vertical curvature                                                 @unit 1/m2                            */
    double      type;                    /**< type of road mark                                                            @unit                                 */
    double      color;                   /**< color of road mark                                                           @unit                                 */
    double      noDataPoints;            /**< number of tesselation points following this package                          @unit _                               */
    double      roadId;                  /**< id of the road to which the roadmark belongs                                 @unit _                               */
    double      spare1;                  /**< for future use                                                               @unit _                               */
} RDB_ROADMARK;

/** ------ signal / sign info for a given vehicle ------ */
typedef struct
{
    double      id;                      /**< ID of the signal                                                             @unit _                               */
    double      playerId;                /**< ID of the player who "detected" the signal                                   @unit _                               */
    double      roadDist;                /**< distance to signal along road                                                @unit m                               */
    double      pos_x;                   /**< position x                                                                   @unit m                               */
    double      pos_y;                   /**< position y                                                                   @unit m                               */
    double      pos_z;                   /**< position z                                                                   @unit m                               */
    double      pos_h;                   /**< heading                                                                      @unit rad                             */
    double      pos_p;                   /**< pitch                                                                        @unit rad                             */
    double      pos_r;                   /**< roll                                                                         @unit rad                             */
    double      type;                    /**< signal type according to OpenDRIVE                                           @unit _                               */
    double      subType;                 /**< signal sub-type according to OpenDRIVE                                       @unit _                               */
    double      value;                   /**< value associated with signal (e.g. speed, mass, validity)                    @unit depending on signal type,       */
    double      state;                   /**< traffic sign's state (if dynamic)                                            @unit _                               */
    double      readability;             /**< sign's readability (-1 = not valid, 0..127 = 0..100% readability)            @unit [-1, 0..127]                    */
    double      occlusion;               /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)  @unit [-1, 0..127]                    */
    double      spare0;                  /**< some more spare                                                              @unit _                               */
    double      addOnId;                 /**< ID of additional sign which extends this sign                                @unit _                               */
    double      minLane;                 /**< ID minimum lane for which sign is valid                                      @unit [-127..127]                     */
    double      maxLane;                 /**< ID maximum lane for which sign is valid                                      @unit [-127..127]                     */
    double      spare;                   /**< spare for upcoming info                                                      @unit _                               */
} RDB_TRAFFIC_SIGN;

/** ------ traffic light information ------ */
typedef struct {
    double      id;                      /**< traffic light id                                                             @unit _                               */
    double      state;                   /**< traffic light remaining time                                                 @unit [0.0..1.0]                      */
    double      stateMask;               /**< traffic light color                              @unit GREEN:0x100000, @YELLOW:0x1000000, @RED:0x10000000          */
    double      ctrlId;                  /**< ID of the traffic light's controller                                         @unit _                               */
    double      cycleTime;               /**< traffic light cycle time                                                     @unit s                               */
    double      dataSize;
    double      noPhases;
} RDB_TRAFFIC_LIGHT;

/** ------ information about an object registered within a sensor ------ */
typedef struct
{
    double      category;                /**< object category                                                              @unit                                 */
    double      type;                    /**< object type                                                                  @unit                                 */
    double      flags;                   /**< sensor object flags                                                          @unit                                 */
    double      id;                      /**< id of the object                                                             @unit _                               */
    double      sensorId;                /**< id of the detecting sensor                                                   @unit _                               */
    double      dist;                    /**< distance between object and referring device                                 @unit m                               */
    double      pos_x;                   /**< position x                                                                   @unit m                               */
    double      pos_y;                   /**< position y                                                                   @unit m                               */
    double      pos_z;                   /**< position z                                                                   @unit m                               */
    double      pos_h;                   /**< heading                                                                      @unit rad                             */
    double      pos_p;                   /**< pitch                                                                        @unit rad                             */
    double      pos_r;                   /**< roll                                                                         @unit rad                             */
    double      occlusion;               /**< degree of occlusion for viewer (-1 = not valid, 0..127 = 0..100% occlusion)  @unit [-1, 0..127]                    */
    double      deltaLanePos;            /**< for future use                                                               @unit _                               */
    double      spare0[2];               /**< for future use                                                               @unit _                               */
    double      spare[3];                /**< for future use                                                               @unit _                               */
} RDB_SENSOR_OBJECT;

typedef union {
    RDB_OBJECT_STATE      perception_obstacles;
    RDB_LANE_INFO         perception_lane_info;
    RDB_ROADMARK          perception_lane_mark;
    RDB_TRAFFIC_SIGN      perception_traffic_sign;
    RDB_TRAFFIC_LIGHT     perception_traffic_light;
    RDB_SENSOR_OBJECT     perception_sensor_object;
} RDB_MSG;
