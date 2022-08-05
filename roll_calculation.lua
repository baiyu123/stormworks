-- This script extract exact roll angle from the tilt and compass sensor data.

pitch = 0
yaw = 0
tilt_left = 0
tilt_down = 0

-- Tick function that will be executed every logic tick
function onTick()
	yaw = input.getNumber(1)			 -- Read the first number from the script's composite input
	pitch = input.getNumber(2)
	tilt_left = input.getNumber(3)
	tilt_down = input.getNumber(4)
	
	if (pitch > 1.4)
	then
		pitch = 1.4
	end
	roll = math.asin(math.sin(tilt_left) / math.sin(1.57-pitch))
	
	-- tilt left wraps around when it is pointing down
	-- tilt down is positive when that happens
	if (tilt_down > 0)
	then
		if (roll > 0)
		then
			roll = roll + 1.57
		else
			roll = roll - 1.57
		end
	end
	
	output.setNumber(1, roll)
end

-- Draw function that will be executed when this script renders to a screen
function onDraw()
	w = screen.getWidth()				  -- Get the screen's width and height
	h = screen.getHeight()					
	screen.setColor(0, 100, 0)			 -- Set draw color to green
	screen.drawText(0, 0, tilt_left)
	screen.drawText(0, 5, roll)
end