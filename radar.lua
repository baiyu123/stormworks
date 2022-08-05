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

function calculateDist(x1, y1, z1, x2, y2, z2)
	dist = math.sqrt((x2-x1)^2 + (y2-y1)^2)
	return dist
end

function scanReset()
	for i, single_target in pairs(targets) do
		scan_azimuth = single_target[1]
		--reset when the scan comes back
		if math.abs(scan_azimuth - rot) < 0.1 and curr_tick - single_target[3] > target_dot_erase_min_time then 
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
new_old_max_dist = 15
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
			dist_lower_bound = calculateDist(gps_x, gps_y, 0, curr_track_target[4], curr_track_target[5], curr_track_target[6])
			dis_min = 999999
		end
		local foundNext = false
		for i, single_target in pairs(targets) do
			local dist = calculateDist(gps_x, gps_y, 0, single_target[4], single_target[5], single_target[6])
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
		for i, single_target in pairs(targets) do
			new_old_dist = calculateDist(curr_track_target[4], curr_track_target[5], curr_track_target[6], single_target[4], single_target[5], single_target[6])
			if new_old_dist < new_old_max_dist then
				curr_track_target = single_target
			end
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

rot_body_radar = -math.pi/2
function rotate(x,y,z,alpha,beta,lambda)
	s_a = sin(alpha)
	c_a = cos(alpha)
	s_b = sin(beta)
	c_b = cos(beta)
	s_l = sin(lambda)
	c_l = cos(lambda)
	new_x = c_a*c_b*x + (c_a*s_b*s_l-s_a*c_l)*y + (c_a*s_b*c_l+s_a*s_l)*z + gps_x
	new_y = s_a*c_b*x + (s_a*s_b*s_l+c_a*c_l)*y + (s_a*s_b*c_l-c_a*s_l)*z + gps_y
	new_z = -s_b*x + c_b*s_l*y + c_b*c_l*z + altitude
	return new_x, new_y, new_z
end

function onTick()
	gps_x = input.getNumber(gpsx_ch)
	gps_y = input.getNumber(gpsy_ch)
	altitude = input.getNumber(alt_ch)
	--inertia frame east counter cw positive
	yaw = scaleToRadian(input.getNumber(compass_ch))
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
				target[0] = input.getNumber(i*3+1)
				-- rotate to align to 0 degree at x axis
				target[1] = (-scaleToRadian(input.getNumber(i*3+2))+math.pi/2)%two_pi
				target[2] = scaleToRadian(input.getNumber(i*3+3))
				target[3] = curr_tick
				dist = target[0]
				-- azimuth_rad_east = target[1]+heading_rad_east+rot_body_radar
				-- azimuth_rad_east = radarToWorld(gps_x,gps_y,altitude, heading_rad_east, 0,0,rot_body_radar)
				target_x_radar = dist*cos(target[1])
				target_y_radar = dist*sin(target[1])
				target_z_radar = dist*sin(target[2])
				target_x, target_y, target_z = rotate(target_x_radar, target_y_radar, target_z_radar, yaw, pitch, roll)
				target[4] = target_x
				target[5] = target_y
				target[6] = target_z
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
		output.setNumber(2, target[4])
		output.setNumber(3, target[5])
		output.setNumber(4, target[6])
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