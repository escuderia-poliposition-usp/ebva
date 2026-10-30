# Classifier observation consumed by the supervisor, not a controller command.
# header is the original observation time in the declared classifier model frame.
std_msgs/Header header
bool valid
# straight, left_curve, right_curve, intersection, walls
string track_type
# left, middle, right
string position
# Finite probabilities in [0,1]; finite nonnegative model entropies.
float64 track_probability
float64 track_entropy
float64 position_probability
float64 position_entropy
# Original scalar feature used by PCurveController; no implicit SI conversion.
float64 curvature_feature
