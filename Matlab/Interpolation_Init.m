function Interpolation_Init(displacement,max_velocity_abs,min_velocity_abs,acceleration_abs,slow_distance_abs)

global acceleration_time const_time deceleration_time slowly_time;
global Distance_Symbols acc_distance const_distance dec_distance slowly_distance;
INTER_FLOAT_DELTA=0.001;
DISATNCE_DELTA=0.3;
 distance_temp = (max_velocity_abs * max_velocity_abs - min_velocity_abs * min_velocity_abs) / acceleration_abs;
 if min_velocity_abs<INTER_FLOAT_DELTA
    slow_distance_abs=0;
 end

Distance_Symbols=sign(displacement);
displacement=displacement*Distance_Symbols;
acceleration_time = 0;
const_time = 0;
deceleration_time = 0;
slowly_time = 0;

acc_distance = 0;
const_distance = 0;
dec_distance = 0;
slowly_distance = 0;

displacement_temp=displacement-slow_distance_abs;
if 
end

end