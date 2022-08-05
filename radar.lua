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
up_range_ch = 9
down_range_ch = 10
lock_target_ch = 11
clear_target_ch = 12

function calculateDist(x1, y1, z1, x2, y2, z2)
	dist = math.sqrt((x2-x1)^2 + (y2-y1)^2)
	return dist
end

function scanReset()
	for i, single_target in pairs(targets) do
		-- 0-1 azimuth 
		azimuth = scaleToRadian(single_target[1])
		--reset when the scan comes back
		if math.cos(azimuth - rot) > 0.95 and curr_tick - single_target[3] > 200 then 
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
	return scale*2*math.pi
end

-- function cos(input)
-- 	return math.cos(input)
-- end

-- function sin(input)
-- 	return math.cos(input)
-- end

-- function radarToWorld(x,y,z,alpha,beta,lambda, radar_yaw_offset, gps_x, gps_y, altitude)
-- 	lambda = lambda + radar_yaw_offset
-- 	new_x = cos(alpha)*cos(beta)*x + 
-- 	(cos(alpha)*sin(beta)*sin(lambda)- sin(alpha)*cos(lambda))*y +
-- 	(cos(alpha)*sin(beta)*cos(lambda) + sin(alpha)*sin(lambda))*z
-- 	new_y = sin(alpha)*cos(beta)*x +
-- 	(sin(alpha)*sin(beta)*sin(lambda)+cos(alpha)*cos(lambda))*y +
-- 	(sin(alpha)*sin(beta)*cos(lambda)-cos(alpha)*sin(lambda))*z
-- 	new_z = -sin(lambda)*x + cos(beta)*sin(lambda)*y + cos(beta)*cos(lambda)*z
-- 	return {new_x, new_y, new_z}
-- end

rot_body_radar = -math.pi/2
function onTick()
	gps_x = input.getNumber(gpsx_ch)
	gps_y = input.getNumber(gpsy_ch)
	--inertia frame east counter cw positive
	heading_rad_east = scaleToRadian(input.getNumber(compass_ch))+math.pi/2
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
				target[1] = -scaleToRadian(input.getNumber(i*3+2))
				target[2] = input.getNumber(i*3+3)
				target[3] = curr_tick
				dist = target[0]
				azimuth_rad_east = target[1]+heading_rad_east+rot_body_radar
				elevation_rad = scaleToRadian(target[2])
				target_x = gps_x + dist*math.cos(azimuth_rad_east)
				target_y = gps_y + dist*math.sin(azimuth_rad_east)
				target_z = dist*math.sin(elevation_rad)
				target[4] = target_x
				target[5] = target_y
				target[6] = target_z
				targets[count] = target
				count = count + 1
			end
		end
	end
	rot = (input.getNumber(yaw_ch)%1)*2*math.pi
	scanReset()
	mergeClosedScan()
	trackTarget()
	output.setNumber(1, radar_range)
	if curr_track_target ~= nil then
		output.setNumber(2, curr_track_target[4])
		output.setNumber(3, curr_track_target[5])
		output.setNumber(4, curr_track_target[1])
	end

	curr_tick = curr_tick + 1
	
end

function onDraw()
	screen.setColor(255, 0, 0)
	local length = 0
	for i, target in pairs(targets) do
		length = length + 1
		dist = target[0]
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