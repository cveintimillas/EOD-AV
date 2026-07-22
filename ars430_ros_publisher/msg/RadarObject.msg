# One tracked object from the radar's internal tracker (SOME/IP service 230).
# Layout reverse-engineered July 2026; rcs and class_id are tentative.
uint8   id          # persistent track id
float32 pos_x       # m, forward
float32 pos_y       # m, lateral (sign convention: see radar_objects invert_y param)
float32 rcs         # dBsm (tentative field)
uint8   prob        # existence probability 0..100
uint8   status      # slot byte 44 (meaning unconfirmed)
uint8   class_id    # slot byte 45 (meaning unconfirmed)
