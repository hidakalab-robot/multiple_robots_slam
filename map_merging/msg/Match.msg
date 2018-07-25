# MatchType
# 0:EigenValue
# 1:SHOT
# 2:Final(Considering all matching result)

std_msgs/Header header
std_msgs/Int8 matchType
map_merging/Cluster sourceMap
map_merging/Cluster mergedMap
map_merging/PairNumber[] matchList
