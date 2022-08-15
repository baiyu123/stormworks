-- Tick function that will be executed every logic tick
radar_range = 1000
radar_ranges = {250, 500, 1000, 2000, 4000}
range_index = 1
pixel_radius = 32
h = 64
w = 64
centerX = w/2
centerY = h/2
targets = {}
curr_tick = 0
count = 0
rot = 0
min_detect_dist = 3
target_cluster_width = 10
gps_x = 0
gps_y = 0
yaw_ch = 25
gpsx_ch = 26
gpsy_ch = 27
compass_ch = 28
alt_ch = 29
pitch_ch = 30
roll_ch = 31
up_range_ch = 9
down_range_ch = 10
lock_target_ch = 11
clear_target_ch = 12
two_pi = 2*math.pi
target_dot_erase_min_time = 100
erase_max_time = 500

function calculateDist(x1, y1, z1, x2, y2, z2)
	dist = math.sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
	return dist
end

function calculateDistNoZ(x1, y1, x2, y2)
	dist = math.sqrt((x2-x1)^2 + (y2-y1)^2)
	return dist
end

function scanReset()
	for i, single_target in pairs(targets) do
		scan_azimuth = single_target[1]
		delta_tick = curr_tick - single_target[3]
		--reset when the scan comes back or exceed max time
		if delta_tick > erase_max_time or (math.abs(scan_azimuth - rot) < 0.1 and delta_tick > target_dot_erase_min_time) then 
			targets[i] = nil
		end
	end
end

function mergeClosedScan()
	-- remove closed targets
	for i, single_target in pairs(targets) do
		-- remove the target that is too closed
		for j, single_target2 in pairs(targets) do
			if j ~= i then
				diff = calculateDist(single_target[4], single_target[5], single_target[6], single_target2[4], single_target2[5], single_target2[6])
				if diff < target_cluster_width then
					-- delete older scan
					if single_target[3] > single_target2[3] then
						targets[j] = nil
					else
						targets[i] = nil
					end
				end
			end
		end
	end
end

previous_up = false
previous_down = false
function checkRange()
	--dealing with range
	if input.getBool(up_range_ch) and not previous_up then
		range_index = range_index + 1
	elseif input.getBool(down_range_ch) and not previous_down then
		range_index = range_index - 1
	end
	previous_up = input.getBool(up_range_ch)
	previous_down = input.getBool(down_range_ch)
	range_index = range_index%5
	radar_range = radar_ranges[range_index+1]
end

prev_lock_pressed = false
curr_track_target = nil
new_old_max_dist = 50
new_old_dist = 999
function trackTarget()
	if input.getBool(clear_target_ch) then
		curr_track_target = nil
	end
	if input.getBool(lock_target_ch) and not prev_lock_pressed then
		local dist_lower_bound = 0
		local min_dist = 999999
		if curr_track_target ~= nil then
			-- get the target whose distance is minimum that greater than lowerbound
			dist_lower_bound = calculateDist(gps_x, gps_y, altitude, curr_track_target[4], curr_track_target[5], curr_track_target[6])
			dis_min = 999999
		end
		local foundNext = false
		for i, single_target in pairs(targets) do
			local dist = calculateDist(gps_x, gps_y, altitude, single_target[4], single_target[5], single_target[6])
			if dist > dist_lower_bound and dist < min_dist then
				min_dist = dist
				curr_track_target = single_target
				foundNext = true
			end
		end
		if not foundNext then
			curr_track_target = nil
		end
	end			
	-- update track target
	if curr_track_target ~= nil then
		closest_new_target = nil
		closest_new_old_dist = 999999
		for i, single_target in pairs(targets) do
			new_old_dist = calculateDist(curr_track_target[4], curr_track_target[5], curr_track_target[6], single_target[4], single_target[5], single_target[6])
			-- only update using new info and with limit
			if new_old_dist < new_old_max_dist and curr_track_target[3] < single_target[3] then
				if new_old_dist < closest_new_old_dist then
					closest_new_target = single_target
					closest_new_old_dist = new_old_dist
				end
			end
		end
		if closest_new_target ~= nil then
			curr_track_target = closest_new_target
		end
	end
	prev_lock_pressed = input.getBool(lock_target_ch)
end
-- -0.5, 0.5 to 0-2pi 
function scaleToRadian(scale)
	return scale*two_pi
end

function cos(input)
	return math.cos(input)
end

function sin(input)
	return math.sin(input)
end

function calculateTrgtPos(roll, pitch, yaw, x_ned, y_ned, z_ned, trgt_dist, trgt_azmth, trgt_elev)
	-- Body FRD frame
	body_x = trgt_dist * math.cos(-trgt_elev) * math.cos(trgt_azmth)
	body_y = trgt_dist * math.cos(-trgt_elev) * math.sin(trgt_azmth)
	body_z = trgt_dist * math.sin(-trgt_elev)
	
	sy = math.sin(yaw)
	sp = math.sin(pitch)
	sr = math.sin(roll)
	cy = math.cos(yaw)
	cp = math.cos(pitch)
	cr = math.cos(roll)

	R11 = cp*cy
	R12 = sr*sp*cy - cr*sy
	R13 = cr*sp*cy + sr*sy
	R21 = cp*sy
	R22 = sr*sp*sy + cr*cy
	R23 = cr*sp*sy - sr*cy
	R31 = -sp
	R32 = sr*cp
	R33 = cr*cp

	-- Target is now in NED frame.
	ned_x = R11 * body_x + R12 * body_y + R13 * body_z + x_ned
	ned_y = R21 * body_x + R22 * body_y + R23 * body_z + y_ned
	ned_z = R31 * body_x + R32 * body_y + R33 * body_z + z_ned
	
	-- Game uses ENU frame, convert to that
	enu_x = ned_y
	enu_y = ned_x
	enu_z = -ned_z
	
	return enu_x, enu_y, enu_z
	end

function get_target_xyz(roll, pitch, yaw, gps_x, gps_y, altitude, trgt_dist, trgt_azmth, trgt_elev)
	return calculateTrgtPos(roll, pitch, yaw, gps_y, gps_x, -altitude, trgt_dist, trgt_azmth*2*math.pi, trgt_elev*2*math.pi)
end


function onTick()
	gps_x = input.getNumber(gpsx_ch)
	gps_y = input.getNumber(gpsy_ch)
	altitude = input.getNumber(alt_ch)
	--inertia frame east counter cw positive
	yaw = -scaleToRadian(input.getNumber(compass_ch))
	pitch = scaleToRadian(input.getNumber(pitch_ch))
	roll = input.getNumber(roll_ch)
	checkRange()
	-- get target data
	for i = 0,7,1
	do
	valid_target = input.getBool(i+1)
		if valid_target then
			target_dist = input.getNumber(i*3+1)
			-- remove target that is too closed and too far
			if target_dist > min_detect_dist and target_dist < radar_range then
				target = {}
				-- target distance, azimuth angle, elevation angle, time since detection
				dist = input.getNumber(i*3+1)
				azmth = input.getNumber(i*3+2)
				elev = input.getNumber(i*3+3)
				target[0] = dist
				-- rotate to align to 0 degree at x axis
				target[1] = (-scaleToRadian(azmth)+math.pi/2)%two_pi
				target[2] = scaleToRadian(elev)
				target[3] = curr_tick
				
				target_x_radar = dist*cos(target[1])
				target_y_radar = dist*sin(target[1])
				target_z_radar = dist*sin(target[2])
				target_x, target_y, target_z = get_target_xyz(roll, pitch, yaw, gps_x, gps_y, altitude, dist, azmth, elev)
				target[4] = target_x
				target[5] = target_y
				target[6] = target_z
				target[7] = azmth
				target[8] = elev
				targets[count] = target
				count = count + 1
			end
		end
	end
	-- rotate to align to 0 degree at x axis
	rot = (-scaleToRadian(input.getNumber(yaw_ch))+0.5*math.pi)%two_pi
	scanReset()
	mergeClosedScan()
	trackTarget()
	output.setNumber(1, radar_range)
	if curr_track_target ~= nil then
		output.setNumber(2, curr_track_target[4])
		output.setNumber(3, curr_track_target[5])
		output.setNumber(4, curr_track_target[6])
		output.setNumber(5, curr_track_target[7])
		output.setNumber(6, curr_track_target[8])
	else
		output.setNumber(2, 0)
		output.setNumber(3, 0)
		output.setNumber(4, 0)
		output.setNumber(5, 0)
		output.setNumber(6, 0)
	end
	curr_tick = curr_tick + 1
	
end

function onDraw()
	screen.setColor(255, 0, 0)
	local length = 0
	for i, target in pairs(targets) do
		length = length + 1
		dist = target[0]
		-- radar facing forward + pi/2
		azimuth = target[1]
		local target_x = centerX + ((dist/radar_range)*pixel_radius)*math.cos(azimuth)
		local target_y = centerY + -((dist/radar_range)*pixel_radius)*math.sin(azimuth)
		screen.drawCircleF(target_x, target_y, 1)
	end
	-- draw track box
	screen.setColor(0,255,0)
	if curr_track_target ~= nil then
		dist = curr_track_target[0]
		azimuth = curr_track_target[1]
		local track_pix_x = centerX + ((dist/radar_range)*pixel_radius)*math.cos(azimuth)
		local track_pix_y = centerY + -((dist/radar_range)*pixel_radius)*math.sin(azimuth)
		screen.drawRect(track_pix_x-1, track_pix_y-1,3,3) 
	end
end