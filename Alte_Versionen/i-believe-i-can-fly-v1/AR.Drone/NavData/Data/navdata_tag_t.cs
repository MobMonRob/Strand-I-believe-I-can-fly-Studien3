﻿
namespace AR.Drone.NavData.Data
{
    public enum navdata_tag_t : ushort
    {
        NAVDATA_DEMO_TAG,
        NAVDATA_TIME_TAG,
        NAVDATA_RAW_MEASURES_TAG,
        NAVDATA_PHYS_MEASURES_TAG,
        NAVDATA_GYROS_OFFSETS_TAG,
        NAVDATA_EULER_ANGLES_TAG,
        NAVDATA_REFERENCES_TAG,
        NAVDATA_TRIMS_TAG,
        NAVDATA_RC_REFERENCES_TAG,
        NAVDATA_PWM_TAG,
        NAVDATA_ALTITUDE_TAG,
        NAVDATA_VISION_RAW_TAG,
        NAVDATA_VISION_OF_TAG,
        NAVDATA_VISION_TAG,
        NAVDATA_VISION_PERF_TAG,
        NAVDATA_TRACKERS_SEND_TAG,
        NAVDATA_VISION_DETECT_TAG,
        NAVDATA_WATCHDOG_TAG,
        NAVDATA_ADC_DATA_FRAME_TAG,
        NAVDATA_VIDEO_STREAM_TAG,
        NAVDATA_GAMES_TAG,
        NAVDATA_PRESSURE_RAW_TAG,
        NAVDATA_MAGNETO_TAG,
        NAVDATA_WIND_TAG,
        NAVDATA_KALMAN_PRESSURE_TAG,
        NAVDATA_HDVIDEO_STREAM_TAG,
        NAVDATA_WIFI_TAG,
        NAVDATA_ZIMMU_3000_TAG,
        NAVDATA_NUM_TAGS,
        NAVDATA_CKS_TAG = 0xffff,
    }
}
