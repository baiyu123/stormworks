radius = 32
w = 64
h = 64
center_x = w/2
center_y = h/2
rot_per_tick = 0.01
rot = 0
range = 100
-- Tick function that will be executed every logic tick
function onTick()
	rot = -input.getNumber(9)*2*math.pi+0.5*math.pi
	line_x = center_x + radius*math.cos(rot)
	line_y = center_y + -radius*math.sin(rot)
end

-- Draw function that will be executed when this script renders to a screen
function onDraw()				
	screen.setColor(0, 255, 0)			 -- Set draw color to gree
	screen.drawCircle(center_x, center_y, radius)   -- Draw a 30px radius circle in the center of the screen
	screen.drawLine(center_x, center_y, line_x, line_y)
end
	
function drawTarget(distance)
	screen.setColor(255, 0, 0)
end