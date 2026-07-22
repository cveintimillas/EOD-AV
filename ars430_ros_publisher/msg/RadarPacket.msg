# One RDI measurement packet (deduplicated: 1 per event per radar cycle)
std_msgs/Header header
uint8  event_id            # 2 = far scan, 4 = near scan
uint32 time_stamp          # radar-internal timestamp (grouping key)
uint32 measurement_counter
float32 vambig
RadarDetection[] detections
