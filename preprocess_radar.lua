-- Tick function that will be executed every logic tick
function onTick()
	for i = 0,7,1 do
		dist = input.getNumber(i*4+1)
		azimuth = input.getNumber(i*4+2)
		elevation = input.getNumber(i*4+3)
		output.setBool(i+1, input.getNumber(i+1))
		output.setNumber(i*3+1, dist)
		output.setNumber(i*3+2, azimuth)
		output.setNumber(i*3+3, elevation)
	end
end

-- Draw function that will be executed when this script renders to a screen
function onDraw()
	
end