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
target_cluster_width = 5

function calculateDist(x1, y1, x2, y2)
	dist = math.sqrt((x2-x1)^2 + (y2-y1)^2)
	return dist
end

function scanReset()
	for i, single_target in pairs(targets) do
		-- 0-1 azimuth 
		azimuth = (single_target[1]/0.5)*math.pi
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
				diff = calculateDist(single_target[4], single_target[5], single_target2[4], single_target2[5])
				if diff < target_cluster_width then
					targets[j] = nil
				end
			end
		end
	end
end

function onTick()
	gps_x = input.getNumber(30)
	gps_y = input.getNumber(31)
	
	--dealing with range
	--if input.getBool(9) then
		range_index = range_index + 1
	--end
	--if input.getBool(10) then
	--	range_index = range_index - 1
	--end
	--range_index = range_index%5 + 1
	--radar_range = radar_ranges[range_index]
	
	-- get target data
	for i = 0,7,1
	do
	valid_target = input.getBool(i)
		if valid_target then
			target_dist = input.getNumber(i*4+1)
			-- remove target that is too closed and too far
			if target_dist > min_detect_dist and input.getNumber(i*4+4) == 1 then
				single_target = {}
				-- target distance, azimuth angle, elevation angle, time since detection
				single_target[0] = input.getNumber(i*4+1)
				single_target[1] = input.getNumber(i*4+2) 
				single_target[2] = input.getNumber(i*4+3)
				single_target[3] = curr_tick
				dist = single_target[0]
				azimuth = (single_target[1]/0.5)*math.pi
				target_x = gps_x + dist*math.cos(azimuth)
				target_y = gps_y + dist*math.sin(azimuth)
				single_target[4] = target_x
				single_target[5] = target_y
				targets[count] = single_target
				count = count + 1
			end
		end
	end
	rot = (input.getNumber(29)%1)*2*math.pi
	scanReset()
	mergeClosedScan()
	curr_tick = curr_tick + 1
	
end

-- Draw function that will be executed when this script renders to a screen
function onDraw()
	screen.setColor(255, 0, 0)
	length = 0
	for i, single_target in pairs(targets) do
		length = length + 1
		dist = single_target[0]
		azimuth = (single_target[1]/0.5)*math.pi
		target_x = centerX + ((dist/radar_range)*pixel_radius)*math.cos(azimuth)
		target_y = centerY + ((dist/radar_range)*pixel_radius)*math.sin(azimuth)
		screen.drawCircleF(target_x, target_y, 1)
		--screen.drawText(0,5, azimuth)
		
	end
	screen.drawText(0, 0, length)
	screen.drawText(0, 5, 'range:')
	screen.drawText(0, 10, radar_range)
end