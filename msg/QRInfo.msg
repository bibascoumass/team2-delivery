string order_id          
bool is_detected    # true if ANY QR code is currently in view
bool is_match   # true if detected QR matches order_id
geometry_msgs/Pose pose     # relative position of QR code for docking
