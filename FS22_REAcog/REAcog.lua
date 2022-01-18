--
-- REA COG Script
-- author: 900Hasse
-- date: 23.11.2021
--
-- V1.0.0.0
--
-----------------------------------------
-- TO DO
---------------
-- 
-- 

-----------------------------------------
-- KNOWN ISSUES
---------------
-- 
-- 

print("---------------------------")
print("----- REA by 900Hasse -----")
print("---------------------------")
REAcog = {};

function REAcog.prerequisitesPresent(specializations)
    return true
end;

function REAcog:update(dt)

	-----------------------------------------------------------------------------------
	-- Debug center of gravity
	-----------------------------------------------------------------------------------
	-- Debug CenterOfMass
	local DebugCOF = false
	if DebugCOF then
		if REAcog.DebugTimer == nil then
			REAcog.DebugTimer = 0;
		end;
		-- Count debug timer
		if REAcog.DebugTimer < 10000 then
			REAcog.DebugTimer = REAcog.DebugTimer + dt;
		else
			REAcog.DebugTimer = 0;
		end;
		-- Get number of vehicles
		local numVehicles = table.getn(g_currentMission.vehicles);
		-- If vehicles present run code
		if numVehicles ~= nil then
			-- Run code for vehicles
			if numVehicles >= 1 then
				for VehicleIndex=1, numVehicles do
					-- Save "vehicle" to local
					local vehicle = g_currentMission.vehicles[VehicleIndex];			
					-- Check if current vehicle exists
					if vehicle ~= nil then
						for _,component in pairs(vehicle.components) do
							-- Get translation
							local wx, wy, wz;
							local COGNode = createTransformGroup("COGNode");
							-- Create nodes for search region
							if REAcog.DebugTimer < 5000 then
								wx, wy, wz = localToWorld(component.node, component.OriginalCogX, component.OriginalCogY, component.OriginalCogZ);
								setTranslation(COGNode, wx, wy, wz);
							else
								local cx, cy, cz = getCenterOfMass(component.node);
								wx, wy, wz = localToWorld(component.node, cx, cy, cz);
								setTranslation(COGNode, wx, wy, wz);
							end;
							DebugUtil.drawDebugNode(COGNode,"X", false)
						end;
						if false then
							if vehicle.spec_wheels ~= nil then
								if vehicle.spec_wheels.wheels ~= nil then 
									local WheelIndex = 0;
									for _,wheel in pairs(vehicle.spec_wheels.wheels) do
										-- Count index for information
										WheelIndex = WheelIndex + 1;
										--Calculate new force point
										local forcePointX,forcePointY,forcePointZ = REAcog:CalcWheelForcePoint(wheel);
										-- Get translation
										local wx, wy, wz = localToWorld(wheel.node, forcePointX, forcePointY, forcePointZ);
										-- Create node for lowest spot
										local ForceNode = createTransformGroup("NodeName");
										-- left or right
										local LeftOrRight = ""
										if wheel.isLeft then
											LeftOrRight = "L";
										else
											LeftOrRight = "R";
										end;
										-- Create nodes for search region							
										setTranslation(ForceNode, wx, wy, wz);
										DebugUtil.drawDebugNode(ForceNode,"WF" .. WheelIndex .. LeftOrRight, false)
										DebugUtil.drawDebugNode(wheel.driveNode,"DN" .. WheelIndex, false)
										DebugUtil.drawDebugNode(wheel.node,"WN" .. WheelIndex, false)
									end;
								end;
							end;
						end;
					end;
				end;
			end;
		end;
	end;
end;


-----------------------------------------------------------------------------------	
-- Calculate new center of mass of vehicle
-----------------------------------------------------------------------------------
function REAcog:CalculateNewCenterOfMass(vehicle)
	-- Check if there are wheels on this vehicle
	if vehicle.spec_wheels ~= nil then
		if vehicle.spec_wheels.wheels ~= nil then 
			-- Print info
			print("----------------------------------------------------------------------------------------")
			print("Name: " .. vehicle:getFullName())
			-- Get number of components
			local NumOfComp = table.getn(vehicle.components);
			local CurrentComp = 0;


			-- Get number of wheels
			local numVehicleWheels = table.getn(vehicle.spec_wheels.wheels);
			-- Adjust center of mass on all components 
			for _,component in pairs(vehicle.components) do
				-- Count to next component
				CurrentComp = CurrentComp + 1;

				-- Get debug values
				local dx, dy, dz = getCenterOfMass(component.node);
				component.OriginalCogX = dx;
				component.OriginalCogY = dy;
				component.OriginalCogZ = dz;

				-- Set status is crawler
				component.IsCrawler = false;

				--Get original values
				cx, cy, cz = getCenterOfMass(component.node);

				-- Get all childs
				local AllChild = {}
				REAcog:GetAllChilds(component.node,AllChild)
				-- Get total number of child
				local TotalNumberOfChild = table.getn(AllChild);

				-- Get wheel center value in relation to component
				local WheelAvarageY = 0;
				local WheelAvarageZ = 0;
				local NumberOfWheel = 0;
				local DistanceWheelsZ = 0;
				local LastWheelZ = 0;
				local LargestWheelRadius = 0;
				local WheelHeight = 0;
				for _,wheel in pairs(vehicle.spec_wheels.wheels) do
					for ChildIndex=1,TotalNumberOfChild do
						ActChild = AllChild[ChildIndex];
						-- Check if this wheel is connected to this component
						if wheel.driveNode == ActChild then
							-- Get translation relative to component
							local WheelX,WheelY,WheelZ = localToLocal(wheel.driveNode, component.node, 0, 0, 0);		
							-- Calculate distance betweene wheels to determine wheel base
							if LastWheelZ == 0 then
								LastWheelZ = WheelZ;
							else
								-- Calculate distance
								local DistanceZ = math.abs(LastWheelZ - WheelZ);
								if DistanceZ > DistanceWheelsZ then
									DistanceWheelsZ = DistanceZ;
								end;
							end;
							-- Add total wheel Y value
							WheelAvarageY = WheelAvarageY + WheelY;
							-- Add total wheel Z value
							WheelAvarageZ = WheelAvarageZ + WheelZ;

							-- Count number of wheels
							NumberOfWheel = NumberOfWheel + 1;

							-- If crawler use rotating parts as reference
							local IsCrawler,LargestRadius,RotHeight = REAcog:GetRotPartFromCrawler(vehicle,component);
							-- Set status is crawler
							component.IsCrawler = IsCrawler and NumberOfWheel > 2;

							-- Get largest radius
							if wheel.radiusOriginal > LargestWheelRadius or LargestRadius > LargestWheelRadius then
								LargestWheelRadius = wheel.radiusOriginal;
							end;
							-- If not crawler use wheel as reference else use rotating part
							if not component.IsCrawler then
								print("Use wheel height=Tires")
								if WheelY < WheelHeight or WheelHeight == 0 then
									WheelHeight = WheelY;
								end;
							else
								print("Use rotating part height=Crawlers")
								if RotHeight > WheelHeight then
									WheelHeight = RotHeight;
								end;
							end;
						end;
					end;
				end;
				-- Calculate average wheel Z value
				if NumberOfWheel > 1 then
					WheelAvarageZ = WheelAvarageZ / NumberOfWheel;
				end;
				-- Calculate average wheel Y value
				if NumberOfWheel > 1 then
					WheelAvarageY = WheelAvarageY / NumberOfWheel;
				end;

				-- Get highest point
				local Highest = 0;
				for ChildIndex=1,TotalNumberOfChild do
					ActChild = AllChild[ChildIndex];
					-- Get translation
					local x,y,z = localToLocal(ActChild, component.node, 0, 0, 0);
					if y > Highest then
						Highest = y;
					end;
				end;

				-- Initialize updates
				if component.UpdateYValue == nil then
					component.UpdateYValue = false;
					component.UpdateZValue = false;
				end;

				-- Choose wheel height, choose lowest wheel if vehicle have got a short wheel base;
				if DistanceWheelsZ > 1 and not component.IsCrawler then
					WheelHeight = WheelAvarageY;
					print("Use average height")
				end;

				-- New center of mass Y value
				local RadiusFactor = 0.4;
				local MinCenterY = 0.05 + WheelHeight;
				local MaxCenterY = math.min(0.3 + WheelHeight,WheelHeight+(LargestWheelRadius*RadiusFactor));
				local NewCenterY = cy;
				if NumberOfWheel > 0 then
					NewCenterY = math.max((Highest - WheelHeight) + WheelHeight,MinCenterY);
					NewCenterY = math.min(NewCenterY,MaxCenterY);
					-- Round value with 2 decimals
					NewCenterY = REAcog:RoundValueTwoDecimals(NewCenterY);
					-- Update Y value
					if NewCenterY > cy then
						component.UpdateYValue = true;
					end;
				end;

				-- New center of mass Z value
				local MaxDistanceToWheels = 1.0;
				if WheelAvarageZ < -0.5 then
					MaxDistanceToWheels = 0.5;
				end;
				local NewCenterZ = cz;
				local DistanceToWheel = 0;
				-- If component has wheels and motor check distance to wheels and adjust
				if NumberOfWheel > 1 and numVehicleWheels > 2 then
					if vehicle.spec_motorized ~= nil then
						if vehicle.spec_motorized.motor ~= nil then
							DistanceToWheel = math.abs(cz - WheelAvarageZ);
							if DistanceToWheel > MaxDistanceToWheels then
								if WheelAvarageZ < 0 then
									NewCenterZ = cz - ((cz - WheelAvarageZ) - MaxDistanceToWheels);
								else
									NewCenterZ = cz - ((cz - WheelAvarageZ) + MaxDistanceToWheels);
								end;
								-- Round value with 2 decimals
								NewCenterZ = REAcog:RoundValueTwoDecimals(NewCenterZ);
								-- Update Z value
								component.UpdateZValue = true;
							end;
						end;
					end;
				end;
				-- Calculate changed COG from confuguration
				local AddX = 0;
				local AddY = 0;
				local AddZ = 0;
				if component.OriginalCOGX ~= nil then
					AddX = REAcog:RoundValueTwoDecimals(cx - component.OriginalCOGX);
					AddY = REAcog:RoundValueTwoDecimals(cy - component.OriginalCOGY);
					AddZ = REAcog:RoundValueTwoDecimals(cz - component.OriginalCOGZ);
				end;

				-- Updates
				local UpdateX = cx;
				local UpdateY = cy;
				local UpdateZ = cz;
				-- Update Y value
				if component.UpdateYValue then
					UpdateY = NewCenterY+AddY;
				end;
				-- Update Z value
				if component.UpdateZValue then
					UpdateZ = NewCenterZ+AddZ;
				end;
				-- Set new center of mass
				print("-----------------")
				print("Component: " .. getName(component.node) .. ", component: " .. CurrentComp .. " of " .. NumOfComp)
				if component.UpdateYValue or component.UpdateZValue then
					setCenterOfMass(component.node, UpdateX, UpdateY, UpdateZ);
					--Print info
					print("Center of mass changed by REA")
					if AddX ~= 0 or AddY ~= 0 or AddZ ~= 0 then
						print("Center of mass, Component XML: X=" .. component.OriginalCOGX .. "m, Y=" .. component.OriginalCOGY .. "m, Z=" .. component.OriginalCOGZ .. "m");
						print("Center of mass, After load finished: X=" .. REAcog:RoundValueTwoDecimals(cx) .. "m, Y=" .. REAcog:RoundValueTwoDecimals(cy) .. "m, Z=" .. REAcog:RoundValueTwoDecimals(cz) .. "m");
						print("Center of mass, adjustments by vehicle customization: X=" .. AddX .. "m, Y=" .. AddY .. "m, Z=" .. AddZ .. "m");
					end;
					print("Y(height) original: " .. REAcog:RoundValueTwoDecimals(cy) .. "m, new: " .. REAcog:RoundValueTwoDecimals(UpdateY) .. "m");
					print("Z(length) original: " .. REAcog:RoundValueTwoDecimals(cz) .. "m, new: " .. REAcog:RoundValueTwoDecimals(UpdateZ) .. "m");
					print("Number of wheels: " .. NumberOfWheel .. ", Wheel height: " .. REAcog:RoundValueTwoDecimals(WheelHeight) .. "m, Largest wheel radius: " .. REAcog:RoundValueTwoDecimals(LargestWheelRadius));
					print("Component Mass: " .. component.mass * 1000 .. "kg");
					if component.IsCrawler then
						print("Component is crawler");
					end;
				else
					print("REA, Center of mass checked but not changed")
				end;
			end;
			print("----------------------------------------------------------------------------------------")
		end;
	end;
end


-----------------------------------------------------------------------------------	
-- Get child nodes, volume and mass of these
-----------------------------------------------------------------------------------
function REAcog:GetAllChilds(Node,AllChild)
	-- Save all child in table
	table.insert(AllChild,Node)
	-- Get number of child
	local NumChild = getNumOfChildren(Node);
	-- If more child to node get data from these as well
	if NumChild > 0 then
		for ChildIndex=0 ,NumChild-1 do
			local ChildNode = getChildAt(Node, ChildIndex);
			REAcog:GetAllChilds(ChildNode,AllChild);
		end;
	end;
end


-----------------------------------------------------------------------------------	
-- Get largets rotating part of crawlers
-----------------------------------------------------------------------------------
function REAcog:GetRotPartFromCrawler(vehicle,component)
	local IsCrawler = false;
	local LargestRadius = 0;
	local RotHeight = 0;
	-- Get all childs
	local AllChild = {}
	REAcog:GetAllChilds(component.node,AllChild)
	-- Get total number of child
	local TotalNumberOfChild = table.getn(AllChild);
	-- Check if component contains crawler wheel
	for _,wheel in pairs(vehicle.spec_wheels.wheels) do
		for ChildIndex=1,TotalNumberOfChild do
			ActChild = AllChild[ChildIndex];
			-- Check if this wheel is connected to this component
			if wheel.driveNode == ActChild then
				if vehicle.spec_crawlers ~= nil then
					if vehicle.spec_crawlers.crawlers ~= nil then
						for _,Crawlers in pairs(vehicle.spec_crawlers.crawlers) do
							if Crawlers.rotatingParts ~= nil then
								-- Save status crawler
								IsCrawler = true;
								-- Get largest rotating part
								for _,RotPart in pairs(Crawlers.rotatingParts) do
									local RotPartX,RotPartY,RotPartZ = localToLocal(RotPart.node, component.node, 0, 0, 0);		
									-- Get largest radius, largest radius is used as height reference
									if RotPart.radius > LargestRadius then
										LargestRadius = RotPart.radius;
										RotHeight = RotPartY;
									end;
								end;
							end;
						end;
					end;
				end;
			end;
		end;
	end;
	-- Return result
	return IsCrawler,LargestRadius,RotHeight;
end


-----------------------------------------------------------------------------------	
-- Function to round value with two decimals
-----------------------------------------------------------------------------------
function REAcog:RoundValueTwoDecimals(x)
	x = x*100;
	x = x>=0 and math.floor(x+0.5) or math.ceil(x-0.5);
	x = x/100;
	return x;
end


-----------------------------------------------------------------------------------	
-- Edited addToPhysics
-----------------------------------------------------------------------------------
function REAcog:addToPhysics()
	-- Check if center of mass should be checked and 
    -- REA
	if self.isServer then
		if not self.isAddedToPhysics then
			if self.COGChangedByREA == nil then
				-- Calculate new center of mass
				REAcog:CalculateNewCenterOfMass(self);
				self.COGChangedByREA = true;
			end;
		end;
	end;
	-- Add original AddToPhysics
	if self.OriginalAddToPhysics == nil then
		self.OriginalAddToPhysics = REAcog.OriginalAddToPhysics;
	end;
	-- Run original function
	return self:OriginalAddToPhysics();
end


-----------------------------------------------------------------------------------	
-- Edited addToPhysics
-----------------------------------------------------------------------------------
function REAcog:loadComponentFromXML(component, xmlFile, key, rootPosition, i)

	-- Add original AddToPhysics
	if self.OriginalloadComponentFromXML == nil then
		self.OriginalloadComponentFromXML = REAcog.OriginalloadComponentFromXML;
	end;
	-- Run original function
	local result = self:OriginalloadComponentFromXML(component, xmlFile, key, rootPosition, i);
	-- Get original center of gravity 
	if component.OriginalCOGX == nil then
		-- Initialize variables
		component.OriginalCOGX = 0;
		component.OriginalCOGY = 0;
		component.OriginalCOGZ = 0;
		-- Get translation
		component.OriginalCOGX, component.OriginalCOGY, component.OriginalCOGZ = getCenterOfMass(component.node);
	end;
	return result;
end


-----------------------------------------------------------------------------------	
-- Calculate force center point
-----------------------------------------------------------------------------------
function REAcog:CalcWheelForcePoint(wheel)
	local TireTypeCRAWLER = 4;
	-- Get position of wheel
	local positionX, positionY, positionZ = wheel.positionX-wheel.directionX*wheel.deltaY, wheel.positionY-wheel.directionY*wheel.deltaY, wheel.positionZ-wheel.directionZ*wheel.deltaY
	-- Get direction of wheel
	local dir = 1
	local WidthOffs = 0;	
	local AdditionalWidth = 0;
	local HeightFactor = 0;
	if wheel.tireType == TireTypeCRAWLER then
		-- Crawlers
		local x,y,z = localToLocal(wheel.repr, wheel.node, 0, 0, 0);
		if x < 0 then
			dir = -1;
		else
			dir = 1;
		end;
		WidthOffs = ((wheel.width*0.95)/2)*dir;	
		HeightFactor = 0.7;
	else
		-- Tires
		if not wheel.isLeft then
			dir = -1
		end
		-- Calculate offset of width and additional wheels
		if wheel.additionalWheels ~= nil then
			local numAdditionalWheels = table.getn(wheel.additionalWheels);
			for AddWheel=1,numAdditionalWheels do
				-- Add width off additional wheels
				AdditionalWidth = AdditionalWidth + (wheel.additionalWheels[AddWheel].width);
			end;
			-- Add offset for the wheel furthest out
			WidthOffs = ((wheel.additionalWheels[numAdditionalWheels].width*0.95)/2)*dir;	
		else
			WidthOffs = ((wheel.width*0.95)/2)*dir;	
		end;
		AdditionalWidth = AdditionalWidth * dir;
		HeightFactor = 0.8;
	end;
	--Calculate force point
	local forcePointX = WidthOffs + AdditionalWidth + wheel.positionX;
	local forcePointY = positionY - wheel.radius * HeightFactor;
	local forcePointZ = positionZ;
	-- Return calculatet position
	return forcePointX,forcePointY,forcePointZ;
end;


-----------------------------------------------------------------------------------	
-- Edited updateWheelBase
-----------------------------------------------------------------------------------
function REAcog:updateWheelBase(wheel)
    if self.isServer and self.isAddedToPhysics then

		-- Add original AddToPhysics
		if self.OriginalupdateWheelBase == nil then
			self.OriginalupdateWheelBase = REAcog.OriginalupdateWheelBase;
		end;
		-- Run original function
		self:OriginalupdateWheelBase(wheel);
		--Calculate new force point
		local forcePointX,forcePointY,forcePointZ = REAcog:CalcWheelForcePoint(wheel);
		-- Set new force point
        setWheelShapeForcePoint(wheel.node, wheel.wheelShape, forcePointX, forcePointY, forcePointZ)

    end
end


if REAcog.ModActivated == nil then

	addModEventListener(REAcog);
	REAcog.ModActivated = true;
	REAcog.FilePath = g_currentModDirectory;
	print("mod activated")

	-- Exchange standard GIANT'S functions for editet by REA
	REAcog.OriginalAddToPhysics = Vehicle.addToPhysics;
	Vehicle.addToPhysics = REAcog.addToPhysics;
	REAcog.OriginalloadComponentFromXML = Vehicle.loadComponentFromXML;
	Vehicle.loadComponentFromXML = REAcog.loadComponentFromXML;

	REAcog.OriginalupdateWheelBase = Wheels.updateWheelBase;
	Wheels.updateWheelBase = REAcog.updateWheelBase;

	-- Standard functions exchanged
	print("New REA functions loaded")

end;
