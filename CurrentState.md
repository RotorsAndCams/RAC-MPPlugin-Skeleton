# CurrentState.cs

## Fields & properties

| Name | Type | Meaning |
| --- | --- | --- |
| firmware | [Firmwares enum](#firmwares-enum) | ??? |
| hilch1 | int | ??? |
| hilch2 | int | ??? |
| hilch3 | int | ??? |
| hilch4 | int | ??? |
| hilch5 | int | ??? |
| hilch6 | int | ??? |
| hilch7 | int | ??? |
| hilch8 | int | ??? |
| lastautowp | int | ??? |
| rcoverridech1 | short | ??? |
| rcoverridech10 | short | ??? |
| rcoverridech11 | short | ??? |
| rcoverridech12 | short | ??? |
| rcoverridech13 | short | ??? |
| rcoverridech14 | short | ??? |
| rcoverridech15 | short | ??? |
| rcoverridech16 | short | ??? |
| rcoverridech17 | short | ??? |
| rcoverridech18 | short | ??? |
| rcoverridech2 | short | ??? |
| rcoverridech3 | short | ??? |
| rcoverridech4 | short | ??? |
| rcoverridech5 | short | ??? |
| rcoverridech6 | short | ??? |
| rcoverridech7 | short | ??? |
| rcoverridech8 | short | ??? |
| rcoverridech9 | short | ??? |
| sensors_enabled | [Mavlink_Sensors bitmask](#mavlink-sensors-bitmask) | ??? |
| sensors_health | [Mavlink_Sensors bitmask](#mavlink-sensors-bitmask) | ??? |
| sensors_present | [Mavlink_Sensors bitmask](#mavlink-sensors-bitmask) | ??? |
| Speech | ISpeech | Text-to-speech object |
| multiplierdist | float | ??? |
| DistanceUnit | string | ??? |
| multiplierspeed | float | ??? |
| SpeedUnit | string | ??? |
| multiplieralt | float | ??? |
| AltUnit | string | ??? |
| rateattitudebackup | int | ??? |
| ratepositionbackup | int | ??? |
| ratestatusbackup | int | ??? |
| ratesensorsbackup | int | ??? |
| ratercbackup | int | ??? |
| KIndexstatic | int | ??? |
| custom_field_names | Dictionary<string, string> | ??? |
| parent | MAVState | ??? |
| prearmstatus | bool | ??? |
| customfield0 | float | ??? |
| customfield1 | float | ??? |
| customfield2 | float | ??? |
| customfield3 | float | ??? |
| customfield4 | float | ??? |
| customfield5 | float | ??? |
| customfield6 | float | ??? |
| customfield7 | float | ??? |
| customfield8 | float | ??? |
| customfield9 | float | ??? |
| customfield10 | float | ??? |
| customfield11 | float | ??? |
| customfield12 | float | ??? |
| customfield13 | float | ??? |
| customfield14 | float | ??? |
| customfield15 | float | ??? |
| customfield16 | float | ??? |
| customfield17 | float | ??? |
| customfield18 | float | ??? |
| customfield19 | float | ??? |
| roll | float | Roll (deg) |
| pitch | float | Pitch (deg) |
| yaw | float | Yaw (deg) |
| SSA | float | SSA (deg) |
| AOA | float | AOA (deg) |
| groundcourse | float | GroundCourse (deg) |
| lat | double | Latitude (dd) |
| lng | double | Longitude (dd) |
| alt | float | Altitude (alt) |
| altasl | float | Altitude (alt) |
| horizondist | float | Horizon Dist (dist) |
| vx | double | Velocity X (ms) |
| vy | double | Velocity Y (ms) |
| vz | double | Velocity Z (ms) |
| vlen | double | ??? |
| altoffsethome | float | Alt Home Offset (dist) |
| gpsstatus | float | Gps Status |
| gpshdop | float | Gps HDOP |
| satcount | float | Sat Count |
| gpsh_acc | float | H Acc (m) |
| gpsv_acc | float | V Acc (m) |
| gpsvel_acc | float | Velocity Accuracy |
| gpshdg_acc | float | Heading Accuracy |
| gpsyaw | float | GPS Yaw (deg) |
| lat2 | double | Latitude2 (dd) |
| lng2 | double | Longitude2 (dd) |
| altasl2 | float | Altitude2 (dist) |
| gpsstatus2 | float | Gps Status2 |
| gpshdop2 | float | Gps HDOP2 |
| satcount2 | float | Sat Count2 |
| groundspeed2 | float | ??? |
| groundcourse2 | float | GroundCourse2 (deg) |
| gpsh_acc2 | float | H Acc2 (m) |
| gpsv_acc2 | float | V Acc2 (m) |
| gpsvel_acc2 | float | Velocity Accuracy |
| gpshdg_acc2 | float | Heading Accuracy |
| gpsyaw2 | float | GPS Yaw (deg) |
| satcountB | float | Sat Count Blend |
| gpstime | DateTime | Current GPS Time |
| altd1000 | float | ??? |
| altd100 | float | ??? |
| airspeed | float | AirSpeed (speed) |
| targetairspeed | float | Airspeed Target (speed) |
| lowairspeed | bool | ??? |
| asratio | float | Airspeed Ratio |
| airspeed1_temp | float | Airspeed1 Temperature |
| airspeed2_temp | float | Airspeed2 Temperature |
| groundspeed | float | GroundSpeed (speed) |
| ax | float | Accel X |
| ay | float | Accel Y |
| az | float | Accel Z |
| accelsq | float | Accel Strength |
| gx | float | Gyro X |
| gy | float | Gyro Y |
| gz | float | Gyro Z |
| gyrosq | float | Gyro Strength |
| mx | float | Mag X |
| my | float | Mag Y |
| mz | float | Mag Z |
| magfield | float | Mag Field |
| imu1_temp | float | IMU1 Temperature |
| ax2 | float | Accel2 X |
| ay2 | float | Accel2 Y |
| az2 | float | Accel2 Z |
| accelsq2 | float | Accel2 Strength |
| gx2 | float | Gyro2 X |
| gy2 | float | Gyro2 Y |
| gz2 | float | Gyro2 Z |
| gyrosq2 | float | Gyro2 Strength |
| mx2 | float | Mag2 X |
| my2 | float | Mag2 Y |
| mz2 | float | Mag2 Z |
| magfield2 | float | Mag2 Field |
| imu2_temp | float | IMU2 Temperature |
| ax3 | float | Accel3 X |
| ay3 | float | Accel3 Y |
| az3 | float | Accel3 Z |
| accelsq3 | float | Accel3 Strength |
| gx3 | float | Gyro3 X |
| gy3 | float | Gyro3 Y |
| gz3 | float | Gyro3 Z |
| gyrosq3 | float | Gyro3 Strength |
| mx3 | float | Mag3 X |
| my3 | float | Mag3 Y |
| mz3 | float | Mag3 Z |
| magfield3 | float | Mag3 Field |
| imu3_temp | float | IMU3 Temperature |
| hygrotemp1 | short | hygrotemp1 (cdegC) |
| hygrohumi1 | ushort | hygrohumi1 (c%) |
| hygrotemp2 | short | hygrotemp2 (cdegC) |
| hygrohumi2 | ushort | hygrohumi2 (c%) |
| ch1in | float | ??? |
| ch2in | float | ??? |
| ch3in | float | ??? |
| ch4in | float | ??? |
| ch5in | float | ??? |
| ch6in | float | ??? |
| ch7in | float | ??? |
| ch8in | float | ??? |
| ch9in | float | ??? |
| ch10in | float | ??? |
| ch11in | float | ??? |
| ch12in | float | ??? |
| ch13in | float | ??? |
| ch14in | float | ??? |
| ch15in | float | ??? |
| ch16in | float | ??? |
| ch1out | float | ??? |
| ch2out | float | ??? |
| ch3out | float | ??? |
| ch4out | float | ??? |
| ch5out | float | ??? |
| ch6out | float | ??? |
| ch7out | float | ??? |
| ch8out | float | ??? |
| ch9out | float | ??? |
| ch10out | float | ??? |
| ch11out | float | ??? |
| ch12out | float | ??? |
| ch13out | float | ??? |
| ch14out | float | ??? |
| ch15out | float | ??? |
| ch16out | float | ??? |
| ch17out | float | ??? |
| ch18out | float | ??? |
| ch19out | float | ??? |
| ch20out | float | ??? |
| ch21out | float | ??? |
| ch22out | float | ??? |
| ch23out | float | ??? |
| ch24out | float | ??? |
| ch25out | float | ??? |
| ch26out | float | ??? |
| ch27out | float | ??? |
| ch28out | float | ??? |
| ch29out | float | ??? |
| ch30out | float | ??? |
| ch31out | float | ??? |
| ch32out | float | ??? |
| esc1_volt | float | ??? |
| esc1_curr | float | ??? |
| esc1_rpm | float | ??? |
| esc1_temp | float | ??? |
| esc2_volt | float | ??? |
| esc2_curr | float | ??? |
| esc2_rpm | float | ??? |
| esc2_temp | float | ??? |
| esc3_volt | float | ??? |
| esc3_curr | float | ??? |
| esc3_rpm | float | ??? |
| esc3_temp | float | ??? |
| esc4_volt | float | ??? |
| esc4_curr | float | ??? |
| esc4_rpm | float | ??? |
| esc4_temp | float | ??? |
| esc5_volt | float | ??? |
| esc5_curr | float | ??? |
| esc5_rpm | float | ??? |
| esc5_temp | float | ??? |
| esc6_volt | float | ??? |
| esc6_curr | float | ??? |
| esc6_rpm | float | ??? |
| esc6_temp | float | ??? |
| esc7_volt | float | ??? |
| esc7_curr | float | ??? |
| esc7_rpm | float | ??? |
| esc7_temp | float | ??? |
| esc8_volt | float | ??? |
| esc8_curr | float | ??? |
| esc8_rpm | float | ??? |
| esc8_temp | float | ??? |
| esc9_volt | float | ??? |
| esc9_curr | float | ??? |
| esc9_rpm | float | ??? |
| esc9_temp | float | ??? |
| esc10_volt | float | ??? |
| esc10_curr | float | ??? |
| esc10_rpm | float | ??? |
| esc10_temp | float | ??? |
| esc11_volt | float | ??? |
| esc11_curr | float | ??? |
| esc11_rpm | float | ??? |
| esc11_temp | float | ??? |
| esc12_volt | float | ??? |
| esc12_curr | float | ??? |
| esc12_rpm | float | ??? |
| esc12_temp | float | ??? |
| ch3percent | float | ??? |
| failsafe | bool | Failsafe |
| rxrssi | int | RX Rssi |
| crit_AOA | float | Crit AOA (deg) |
| lowgroundspeed | bool | ??? |
| verticalspeed | float | Vertical Speed (speed) |
| verticalspeed_fpm | double | Vertical Speed (fpm) |
| glide_ratio | double | Glide Ratio |
| nav_roll | float | Roll Target (deg) |
| nav_pitch | float | Pitch Target (deg) |
| nav_bearing | float | Bearing Target (deg) |
| target_bearing | float | Bearing Target (deg) |
| wp_dist | float | Dist to WP (dist) |
| alt_error | float | Altitude Error (dist) |
| ber_error | float | Bearing Error (deg) |
| aspd_error | float | Airspeed Error (speed) |
| xtrack_error | float | Xtrack Error (m) |
| wpno | float | WP No |
| mode | string | Mode |
| climbrate | float | ClimbRate (speed) |
| tot | int | Time over Target (sec) |
| toh | int | Time over Home (sec) |
| distTraveled | float | Dist Traveled (dist) |
| timeSinceArmInAir | float | Time in Air (sec) |
| timeInAir | float | Time in Air (sec) |
| timeInAirMinSec | float | Time in Air (min.sec) |
| turnrate | float | Turn Rate (speed) |
| turng | float | Turn Gs (load) |
| radius | float | Turn Radius (dist) |
| QNH | float | ??? |
| wind_dir | float | Wind Direction (Deg) |
| wind_vel | float | Wind Velocity (speed) |
| targetaltd100 | float | ??? |
| targetalt | float | ??? |
| messages | List<(DateTime time, string message)> | ??? |
| message | string | ??? |
| messageHigh | string | ??? |
| messageHighSeverity | [MAV_SEVERITY enum](#mav_severity-enum) | ??? |
| battery_voltage | double | Bat Voltage (V) |
| battery_voltage2 | double | Bat2 Voltage (V) |
| battery_voltage3 | double | Bat3 Voltage (V) |
| battery_voltage4 | double | Bat4 Voltage (V) |
| battery_voltage5 | double | Bat5 Voltage (V) |
| battery_voltage6 | double | Bat6 Voltage (V) |
| battery_voltage7 | double | Bat7 Voltage (V) |
| battery_voltage8 | double | Bat8 Voltage (V) |
| battery_voltage9 | double | Bat9 Voltage (V) |
| battery_remaining | int | Bat Remaining (%) |
| battery_remaining2 | int | Bat2 Remaining (%) |
| battery_remaining3 | int | Bat3 Remaining (%) |
| battery_remaining4 | int | Bat4 Remaining (%) |
| battery_remaining5 | int | Bat5 Remaining (%) |
| battery_remaining6 | int | Bat6 Remaining (%) |
| battery_remaining7 | int | Bat7 Remaining (%) |
| battery_remaining8 | int | Bat8 Remaining (%) |
| battery_remaining9 | int | Bat9 Remaining (%) |
| current | double | Bat Current (Amps) |
| current2 | double | Bat2 Current (Amps) |
| current3 | double | Bat3 Current (Amps) |
| current4 | double | Bat4 Current (Amps) |
| current5 | double | Bat5 Current (Amps) |
| current6 | double | Bat6 Current (Amps) |
| current7 | double | Bat7 Current (Amps) |
| current8 | double | Bat8 Current (Amps) |
| current9 | double | Bat9 Current (Amps) |
| watts | double | Bat Watts |
| battery_mahperkm | double | Bat efficiency (mah/km) |
| battery_kmleft | double | Bat km left EST (km) |
| battery_cell1 | double | ??? |
| battery_cell2 | double | ??? |
| battery_cell3 | double | ??? |
| battery_cell4 | double | ??? |
| battery_cell5 | double | ??? |
| battery_cell6 | double | ??? |
| battery_cell7 | double | ??? |
| battery_cell8 | double | ??? |
| battery_cell9 | double | ??? |
| battery_cell10 | double | ??? |
| battery_cell11 | double | ??? |
| battery_cell12 | double | ??? |
| battery_cell13 | double | ??? |
| battery_cell14 | double | ??? |
| battery_temp | double | ??? |
| battery_temp2 | double | ??? |
| battery_temp3 | double | ??? |
| battery_temp4 | double | ??? |
| battery_temp5 | double | ??? |
| battery_temp6 | double | ??? |
| battery_temp7 | double | ??? |
| battery_temp8 | double | ??? |
| battery_temp9 | double | ??? |
| battery_remainmin | double | ??? |
| battery_remainmin2 | double | ??? |
| battery_remainmin3 | double | ??? |
| battery_remainmin4 | double | ??? |
| battery_remainmin5 | double | ??? |
| battery_remainmin6 | double | ??? |
| battery_remainmin7 | double | ??? |
| battery_remainmin8 | double | ??? |
| battery_remainmin9 | double | ??? |
| battery_usedmah | double | Bat used EST (mah) |
| battery_usedmah2 | double | Bat2 used EST (mah) |
| battery_usedmah3 | double | Bat3 used EST (mah) |
| battery_usedmah4 | double | Bat4 used EST (mah) |
| battery_usedmah5 | double | Bat5 used EST (mah) |
| battery_usedmah6 | double | Bat6 used EST (mah) |
| battery_usedmah7 | double | Bat7 used EST (mah) |
| battery_usedmah8 | double | Bat8 used EST (mah) |
| battery_usedmah9 | double | Bat9 used EST (mah) |
| HomeAlt | double | ??? |
| HomeLocation | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| PlannedHomeLocation | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| MovingBase | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| TrackerLocation | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| Location | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| TargetLocation | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| GeoFenceDist | float | ??? |
| DistToHome | float | Dist to Home (dist) |
| DistFromMovingBase | float | Dist to Moving Base (dist) |
| ELToMAV | float | Elevation to Mav (deg) |
| AZToMAV | float | Bearing to Mav (deg) |
| sonarrange | float | Sonar Range (alt) |
| sonarvoltage | float | Sonar Voltage (Volt) |
| rangefinder1 | uint | RangeFinder1 (cm) |
| rangefinder2 | uint | RangeFinder2 (cm) |
| rangefinder3 | uint | RangeFinder3 (cm) |
| freemem | float | ??? |
| load | float | ??? |
| brklevel | float | ??? |
| armed | bool | ??? |
| rssi | float | Sik Radio rssi |
| remrssi | float | Sik Radio remote rssi |
| txbuffer | byte | ??? |
| noise | float | Sik Radio noise |
| remnoise | float | Sik Radio remote noise |
| rxerrors | ushort | ??? |
| fixedp | ushort | ??? |
| localsnrdb | float | Sik Radio snr |
| remotesnrdb | float | Sik Radio remote snr |
| DistRSSIRemain | float | Sik Radio est dist (m) |
| packetdropremote | ushort | ??? |
| linkqualitygcs | ushort | ??? |
| errors_count1 | ushort | Error Type |
| errors_count2 | ushort | Error Type |
| errors_count3 | ushort | Error Type |
| errors_count4 | ushort | Error Count |
| hwvoltage | float | HW Voltage |
| boardvoltage | float | Board Voltage |
| servovoltage | float | Servo Rail Voltage |
| voltageflag | uint | Voltage Flags |
| i2cerrors | ushort | ??? |
| timesincelastshot | double | ??? |
| press_abs | float | ??? |
| press_temp | int | ??? |
| press_abs2 | float | ??? |
| press_temp2 | int | ??? |
| rateattitude | int | ??? |
| rateposition | int | ??? |
| ratestatus | int | ??? |
| ratesensors | int | ??? |
| raterc | int | ??? |
| datetime | DateTime | Reference DateTime, used for comparisons |
| connected | bool | ??? |
| campointa | float | ??? |
| campointb | float | ??? |
| campointc | float | ??? |
| GimbalPoint | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| gimballat | float | ??? |
| gimballng | float | ??? |
| landed | bool | ??? |
| safteyactive | bool | ??? |
| terrainactive | bool | ??? |
| ter_curalt | float | Terrain AGL |
| ter_alt | float | Terrain GL |
| ter_load | float | ??? |
| ter_pend | float | ??? |
| ter_space | float | ??? |
| KIndex | int | ??? |
| opt_m_x | float | flow_comp_m_x |
| opt_m_y | float | flow_comp_m_y |
| opt_x | short | flow_x |
| opt_y | short | flow_y |
| opt_qua | byte | flow quality |
| ekfstatus | float | ??? |
| ekfflags | int | ??? |
| ekfvelv | float | ??? |
| ekfcompv | float | ??? |
| ekfposhor | float | ??? |
| ekfposvert | float | ??? |
| ekfteralt | float | ??? |
| pidff | float | ??? |
| pidP | float | ??? |
| pidI | float | ??? |
| pidD | float | ??? |
| pidaxis | byte | ??? |
| piddesired | float | ??? |
| pidachieved | float | ??? |
| pidSRate | float | ??? |
| pidPDmod | float | ??? |
| vibeclip0 | uint | ??? |
| vibeclip1 | uint | ??? |
| vibeclip2 | uint | ??? |
| vibex | float | ??? |
| vibey | float | ??? |
| vibez | float | ??? |
| version | Version | Version number (major, minor, build, revision) |
| uid | long | ??? |
| uid2 | string | ??? |
| rpm1 | float | ??? |
| rpm2 | float | ??? |
| capabilities | uint | ??? |
| speedup | float | ??? |
| vtol_state | byte | ??? |
| landed_state | byte | ??? |
| gen_status | float | ??? |
| gen_speed | float | ??? |
| gen_current | float | ??? |
| gen_voltage | float | ??? |
| gen_runtime | uint | ??? |
| gen_maint_time | int | ??? |
| efi_baro | float | EFI Baro Pressure (kPa) |
| efi_headtemp | float | EFI Head Temp (C) |
| efi_load | float | EFI Load (%) |
| efi_health | byte | EFI Health |
| efi_exhasttemp | float | EFI Exhast Temp (C) |
| efi_intaketemp | float | EFI Intake Temp (C) |
| efi_rpm | float | EFI rpm |
| efi_fuelflow | float | EFI Fuel Flow (g/min) |
| efi_fuelconsumed | float | EFI Fuel Consumed (g) |
| efi_fuelpressure | float | EFI Fuel Pressure (kPa) |
| xpdr_es1090_tx_enabled | bool | Transponder 1090ES Tx Enabled |
| xpdr_mode_S_enabled | bool | Transponder Mode S Reply Enabled |
| xpdr_mode_C_enabled | bool | Transponder Mode C Reply Enabled |
| xpdr_mode_A_enabled | bool | Transponder Mode A Reply Enabled |
| xpdr_ident_active | bool | Ident Active |
| xpdr_x_bit_status | bool | X-bit Status |
| xpdr_interrogated_since_last | bool | Interrogated since last |
| xpdr_airborne_status | bool | Airborne |
| xpdr_mode_A_squawk_code | ushort | Transponder Mode A squawk code |
| xpdr_nic | byte | NIC |
| xpdr_nacp | byte | NACp |
| xpdr_board_temperature | byte | Board Temperature in C |
| xpdr_maint_req | bool | Maintainence Required |
| xpdr_adsb_tx_sys_fail | bool | ADSB Tx System Failure |
| xpdr_gps_unavail | bool | GPS Unavailable |
| xpdr_gps_no_fix | bool | GPS No Fix |
| xpdr_status_unavail | bool | Ping200X No Status Message Recieved |
| xpdr_status_pending | bool | Status Update Pending |
| xpdr_flight_id | byte[] | Callsign/Flight ID |
| fenceb_count | ushort | Breach count |
| fenceb_status | byte | Breach status |
| fenceb_type | byte | Breach type |
| posn | float | North |
| pose | float | East |
| posd | float | Down |

> [Back to top](#top)

### Firmwares enum:

- ArduPlane
- ArduCopter2
- ArduRover
- ArduSub
- Ateryx
- ArduTracker
- Gimbal
- PX4
- AP_Periph
- Other

> [Back to top](#top)

### Mavlink sensors bitmask:

| Name | Bitmask position | Meaning |
| --- | --- | --- |
| gyro | 1 | 3D gyro |
| accelerometer | 2 | 3D accelerometer |
| compass | 4 | 3D magnetometer |
| barometer | 8 | absolute pressure |
| differential_pressure | 16 | differential pressure |
| gps | 32 | GPS |
| optical_flow | 64 | optical flow |
| VISION_POSITION | 128 | computer vision position |
| LASER_POSITION | 256 | laser based position |
| GROUND_TRUTH | 512 | external ground truth (Vicon or Leica) |
| rate_control | 1024 | 3D angular rate control |
| attitude_stabilization | 2048 | attitude stabilization |
| yaw_position | 4096 | yaw position |
| altitude_control | 8192 | z/altitude control |
| xy_position_control | 16384 | x/y position control |
| motor_control | 32768 | motor outputs / control |
| rc_receiver | 65536 | rc receiver |
| gyro2 | 131072 | 2nd 3D gyro |
| accel2 | 262144 | 2nd 3D accelerometer |
| mag2 | 524288 | 2nd 3D magnetometer |
| geofence | 1048576 | geofence |
| ahrs | 2097152 | AHRS subsystem health |
| terrain | 4194304 | Terrain subsystem health |
| revthrottle | 8388608 | Motors are reversed |
| logging | 16777216 | Logging |
| battery | 33554432 | Battery |
| proximity | 67108864 | Proximity |
| satcom | 134217728 | Satellite Communication |
| prearm | 268435456 | pre-arm check status. Always healthy when armed |

> [Back to top](#top)

### MAV_SEVERITY enum:

| Value | Meaning |
| --- | --- |
| EMERGENCY | System is unusable. This is a 'panic' condition. |
| ALERT | Action should be taken immediately. Indicates error in non-critical systems. |
| CRITICAL | Action must be taken immediately. Indicates failure in a primary system. |
| ERROR | Indicates an error in secondary/redundant systems. |
| WARNING | Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. |
| NOTICE | An unusual event has occurred, though not an error condition. This should be investigated for the root cause. |
| INFO | Normal operational messages. Useful for logging. No action is required for these messages. |
| DEBUG | Useful non-operational messages that can assist in debugging. These should not occur during normal operation. |

> [Back to top](#top)

### PointLatLngAlt class fields:

| Name | Type | Meaning |
| --- | --- | --- |
| Zero | [PointLatLngAlt](#pointlatlngalt-class-fields) | ??? |
| Lat | double | ??? |
| Lng | double | ??? |
| Alt | double | ??? |
| Tag | string | ??? |
| Tag2 | string | ??? |
| color | Color | ??? |

**Class also contains various calculation functions**

> [Back to top](#top)
