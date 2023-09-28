--
-- REA COG Script
-- author: 900Hasse
-- date: 23.11.2021
--
-- V1.0.1.2
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
	local DebugCOF = false;
	if DebugCOF then
		if REAcog.DebugTimer == nil then
			REAcog.DebugTimer = 0;
		end;
		-- Count debug timer
		if REAcog.DebugTimer < 5000 then
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
							local COGNoAddMassNode = createTransformGroup("COGNoAddMassNode");
							local COGAddMassNode = createTransformGroup("COGAddMassNodeNode");
							-- Create nodes for search region
						--	if REAcog.DebugTimer < 2500 and  component.OriginalCogX ~= nil then
						--		wx, wy, wz = localToWorld(component.node, component.OriginalCogX, component.OriginalCogY, component.OriginalCogZ);
						--		setTranslation(COGNode, wx, wy, wz);
						--	else
								local cx, cy, cz = getCenterOfMass(component.node);
								wx, wy, wz = localToWorld(component.node, cx, cy, cz);
								setTranslation(COGNode, wx, wy, wz);
								if component.ReaCOGX ~= nil then
									wx, wy, wz = localToWorld(component.node, component.ReaCOGX, component.ReaCOGY, component.ReaCOGZ);
									setTranslation(COGNoAddMassNode, wx, wy, wz);
								end;
								if component.AdditionalMassCOGX ~= nil then
									wx, wy, wz = localToWorld(component.node, component.AdditionalMassCOGX, component.AdditionalMassCOGY, component.AdditionalMassCOGZ);
									setTranslation(COGAddMassNode, wx, wy, wz);
								end;
						--	end;
							DebugUtil.drawDebugNode(COGNode,"C", false)
							if component.ReaCOGX ~= nil then
								DebugUtil.drawDebugNode(COGNoAddMassNode,"D", false)
							end;
							if component.AdditionalMassCOGX ~= nil then
								DebugUtil.drawDebugNode(COGAddMassNode,"A", false)
							end;
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
			REAcog:PrintDebug("----------------------------------------------------------------------------------------")
			REAcog:PrintDebug("Name: " .. vehicle:getFullName())
			-- Get number of components
			local NumOfComp = table.getn(vehicle.components);
			local CurrentComp = 0;
			local motorized_vehicle = false
			if vehicle.spec_motorized ~= nil then
				if vehicle.spec_motorized.motor ~= nil then
					motorized_vehicle = true
				end;
			end

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
								REAcog:PrintDebug("Use wheel height=Tires")
								if WheelY < WheelHeight or WheelHeight == 0 then
									WheelHeight = WheelY;
								end;
							else
								REAcog:PrintDebug("Use rotating part height=Crawlers")
								if RotHeight > WheelHeight then
									WheelHeight = RotHeight;
								end;
							end;
							-- Keep motorized_vehicle information at wheel level
							wheel.REAcog_motorized_vehicle = motorized_vehicle
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
					REAcog:PrintDebug("Use average height")
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
				component.ReaCOGX = cx;
				component.ReaCOGY = cy;
				component.ReaCOGZ = cz;
				-- Update Y value
				if component.UpdateYValue then
					component.ReaCOGY = NewCenterY+AddY;
				end;
				-- Update Z value
				if component.UpdateZValue then
					component.ReaCOGZ = NewCenterZ+AddZ;
				end;
				-- Set new center of mass
				REAcog:PrintDebug("-----------------")
				REAcog:PrintDebug("Component: " .. getName(component.node) .. ", component: " .. CurrentComp .. " of " .. NumOfComp)
				if component.UpdateYValue or component.UpdateZValue then
					setCenterOfMass(component.node, component.ReaCOGX, component.ReaCOGY, component.ReaCOGZ);
					--Print info
					REAcog:PrintDebug("Center of mass changed by REA")
					if AddX ~= 0 or AddY ~= 0 or AddZ ~= 0 then
						REAcog:PrintDebug("Center of mass, Component XML: X=" .. component.OriginalCOGX .. "m, Y=" .. component.OriginalCOGY .. "m, Z=" .. component.OriginalCOGZ .. "m");
						REAcog:PrintDebug("Center of mass, After load finished: X=" .. REAcog:RoundValueTwoDecimals(cx) .. "m, Y=" .. REAcog:RoundValueTwoDecimals(cy) .. "m, Z=" .. REAcog:RoundValueTwoDecimals(cz) .. "m");
						REAcog:PrintDebug("Center of mass, adjustments by vehicle customization: X=" .. AddX .. "m, Y=" .. AddY .. "m, Z=" .. AddZ .. "m");
					end;
					REAcog:PrintDebug("Y(height) original: " .. REAcog:RoundValueTwoDecimals(cy) .. "m, new: " .. REAcog:RoundValueTwoDecimals(component.ReaCOGY) .. "m");
					REAcog:PrintDebug("Z(length) original: " .. REAcog:RoundValueTwoDecimals(cz) .. "m, new: " .. REAcog:RoundValueTwoDecimals(component.ReaCOGZ) .. "m");
					REAcog:PrintDebug("Number of wheels: " .. NumberOfWheel .. ", Wheel height: " .. REAcog:RoundValueTwoDecimals(WheelHeight) .. "m, Largest wheel radius: " .. REAcog:RoundValueTwoDecimals(LargestWheelRadius));
					REAcog:PrintDebug("Component Mass: " .. component.mass * 1000 .. "kg");
					if component.IsCrawler then
						REAcog:PrintDebug("Component is crawler");
					end;
				else
					REAcog:PrintDebug("REA, Center of mass checked but not changed")
				end;
			end;
			REAcog:PrintDebug("----------------------------------------------------------------------------------------")
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
	local WidthOffFactor = 0.95;
	if wheel.tireType == TireTypeCRAWLER then
		-- Crawlers
		local x,y,z = localToLocal(wheel.repr, wheel.node, 0, 0, 0);
		if x < 0 then
			dir = -1;
		else
			dir = 1;
		end;
		WidthOffs = ((wheel.width*WidthOffFactor)/2)*dir;	
		HeightFactor = 0.7;
	else
		-- Tires
		if not wheel.isLeft then
			dir = -1
		end
		-- Calculate offset of width and additional wheels
		if not wheel.REAcog_motorized_vehicle then
			-- if implement (not motorized vehicle) wheel WidthOffFactor = 0.1
			WidthOffFactor = 0.1
			REAcog:PrintDebug("Implement wheel " .. tostring(wheel.tireFilename))
		end
		if wheel.additionalWheels ~= nil then
			local numAdditionalWheels = table.getn(wheel.additionalWheels);
			for AddWheel=1,numAdditionalWheels do
				-- Add width off additional wheels
				AdditionalWidth = AdditionalWidth + (wheel.additionalWheels[AddWheel].width);
			end;
			-- Add offset for the wheel furthest out
			WidthOffs = ((wheel.additionalWheels[numAdditionalWheels].width*WidthOffFactor)/2)*dir;	
		else
			WidthOffs = ((wheel.width*WidthOffFactor)/2)*dir;	
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


-----------------------------------------------------------------------------------	
-- Edited updateMass
-----------------------------------------------------------------------------------
function REAcog:updateMass()
    self.serverMass = 0
    for _, component in ipairs(self.components) do
        if component.defaultMass == nil then
            if component.isDynamic then
                component.defaultMass = getMass(component.node)
            else
                component.defaultMass = 1
            end
            component.mass = component.defaultMass
        end

        local mass = self:getAdditionalComponentMass(component)
        component.mass = component.defaultMass + mass
        self.serverMass = self.serverMass + component.mass
    end
    local realTotalMass = 0
    for _, component in ipairs(self.components) do
        realTotalMass = realTotalMass + self:getComponentMass(component)
    end
    self.precalculatedMass = realTotalMass - self.serverMass
    for _, component in ipairs(self.components) do
        local maxFactor = self.serverMass / (self.maxComponentMass - self.precalculatedMass)
        if maxFactor > 1 then
            component.mass = component.mass / maxFactor
        end
		--------------
		-- REA Edit --
		--------------		
        -- only update physically mass if difference to last mass is greater 20kg or beeing forced from additional mass calculation
		if component.REAUpdateCOG == nil then
			component.REAUpdateCOG = false;
		end;
        if self.isServer and component.isDynamic and (math.abs(component.lastMass-component.mass) > 0.02 or component.REAUpdateCOG) then
            setMass(component.node, component.mass)
            component.lastMass = component.mass
			if component.ReaCOGX ~= nil and component.ReaCOGY ~= nil and component.ReaCOGZ ~= nil and component.AdditionalMassCOGX ~= nil and component.AdditionalMassCOGY ~= nil and component.AdditionalMassCOGZ ~= nil then
				local COGX = REAcog:GetCenterOfMassTwoObjects(component.ReaCOGX,component.defaultMass,component.AdditionalMassCOGX,component.mass-component.defaultMass);
				local COGY = REAcog:GetCenterOfMassTwoObjects(component.ReaCOGY,component.defaultMass,component.AdditionalMassCOGY,component.mass-component.defaultMass);
				local COGZ = REAcog:GetCenterOfMassTwoObjects(component.ReaCOGZ,component.defaultMass,component.AdditionalMassCOGZ,component.mass-component.defaultMass);
				setCenterOfMass(component.node, COGX, COGY, COGZ);
				component.REAUpdateCOG = false;
			end;
        end
		--------------
    end
    self.serverMass = math.min(self.serverMass, self.maxComponentMass - self.precalculatedMass)
end

-----------------------------------------------------------------------------------	
-- Edited getAdditionalComponentMass
-----------------------------------------------------------------------------------
function REAcog:getAdditionalComponentMass(superFunc, component)
    local additionalMass = superFunc(self, component)
    local spec = self.spec_fillUnit
	--------------
	-- REA Edit --
	--------------
	if component.ReaCOGX ~= nil and component.ReaCOGY ~= nil and component.ReaCOGZ ~= nil then
		component.AdditionalMassCOGX = component.ReaCOGX;
		component.AdditionalMassCOGY = component.ReaCOGY;
		component.AdditionalMassCOGZ = component.ReaCOGZ;
	else
		component.AdditionalMassCOGX, component.AdditionalMassCOGY, component.AdditionalMassCOGZ = getCenterOfMass(component.node);
	end;
	
    for _, fillUnit in ipairs(spec.fillUnits) do
        if fillUnit.updateMass and fillUnit.fillMassNode == component.node and fillUnit.fillType ~= nil and fillUnit.fillType ~= FillType.UNKNOWN then
            local desc = g_fillTypeManager:getFillTypeByIndex(fillUnit.fillType)
            local mass = fillUnit.fillLevel * desc.massPerLiter
			--------------
			-- REA Edit --
			--------------
			if mass > 1 then
				local AddMassCOGX, AddMassCOGY, AddMassCOGZ = 0,0,0;
				if fillUnit.fillType ~= FillType.DIESEL and fillUnit.fillType ~= FillType.DEF then
					local t = self:getFillUnitFillLevelPercentage(fillUnit.fillUnitIndex)
					local FillVolumeFound = false;
					if self.spec_fillVolume ~= nil then
						local fillVolumes = self.spec_fillVolume
						for _, fillVolume in ipairs(fillVolumes.volumes) do
							if fillUnit.fillUnitIndex == fillVolume.fillUnitIndex then
								FillVolumeFound = true;
								local pX,_,pZ,R = getShapeBoundingSphere(fillVolume.volume);
								local x,y,z = localToLocal(fillVolume.baseNode, component.node,pX,0,pZ);
								AddMassCOGX = x;
								AddMassCOGY = (getFillPlaneHeightAtLocalPos(fillVolume.volume, pX, pZ) / 2) + y + fillVolume.heightOffset;
								AddMassCOGZ = z;
								-- Debug
								--DebugUtil.drawDebugNode(fillVolume.baseNode,"X=" .. pX .. " Z=" .. pZ .. " R=" .. R, false);
							end;
						end;
					end;
					if not FillVolumeFound then
						-- If there is no fill volume, use generic value for example liquids
						local MaxHeight = 0.5;
						if fillUnit.capacity > 0 then
							MaxHeight = MathUtil.clamp(fillUnit.capacity / 4000, 0.25, 1.25);
						end;
						if component.ReaCOGX ~= nil then
							AddMassCOGX = component.ReaCOGX;
							AddMassCOGY = component.ReaCOGY;
							AddMassCOGZ = component.ReaCOGZ;
						else
							AddMassCOGX, AddMassCOGY, AddMassCOGZ = getCenterOfMass(component.node);
						end;
						AddMassCOGY = (MaxHeight * t) + AddMassCOGY;
				--		-- Debug
				--		DebugUtil.drawDebugNode(fillUnit.fillMassNode,"T " .. REAcog:RoundValueTwoDecimals(self:getFillUnitFillLevelPercentage(fillUnit.fillUnitIndex)), false)
					end;
					component.REAUpdateCOG = true;
				else
					-- If diesel or DEF, add mass att center of gravity
					if component.ReaCOGX ~= nil then
						AddMassCOGX = component.ReaCOGX;
						AddMassCOGY = component.ReaCOGY;
						AddMassCOGZ = component.ReaCOGZ;
					else
						AddMassCOGX, AddMassCOGY, AddMassCOGZ = getCenterOfMass(component.node);
					end;
				end;
				component.AdditionalMassCOGX = REAcog:GetCenterOfMassTwoObjects(component.AdditionalMassCOGX,additionalMass,AddMassCOGX,mass);
				component.AdditionalMassCOGY = REAcog:GetCenterOfMassTwoObjects(component.AdditionalMassCOGY,additionalMass,AddMassCOGY,mass);
				component.AdditionalMassCOGZ = REAcog:GetCenterOfMassTwoObjects(component.AdditionalMassCOGZ,additionalMass,AddMassCOGZ,mass);
			end;
			--------------
            additionalMass = additionalMass + mass
        end
    end
    return additionalMass
end


-----------------------------------------------------------------------------------	
-- Calculate the center of mass given two center of mass
-----------------------------------------------------------------------------------
function REAcog:GetCenterOfMassTwoObjects(P1,M1,P2,M2)
	if M1 > 0 or M2 > 0 then
		return ((P1 * M1) + (P2 * M2)) / (M1 + M2);
	end;
end;


-----------------------------------------------------------------------------------	
-- Edited onUpdateTick
-----------------------------------------------------------------------------------
function REAcog:onUpdateTick(dt, isActiveForInput, isActiveForInputIgnoreSelection, isSelected)
    local spec = self.spec_fillUnit
    if self.isServer then
		-- REA Upate
		----------------------------------------
		if self:getIsActive() and not self.isMassDirty then
			local UpdateRate = 500;
			-- Initialize timer
			if spec.MassUpdateTimer == nil then
				spec.MassUpdateTimer = UpdateRate;
			end;
			-- Update Timer
			if spec.MassUpdateTimer < 0 then
				spec:getFullName()
				--print(spec:getFullName() .. " = SetMass");
				self:setMassDirty()
				spec.MassUpdateTimer = UpdateRate;
			else
				spec.MassUpdateTimer = spec.MassUpdateTimer - dt;
			end;
		end;
		----------------------------------------
		if spec.fillTrigger.isFilling then
			local delta = 0
			local trigger = spec.fillTrigger.currentTrigger
			if trigger ~= nil then
				delta = spec.fillTrigger.litersPerSecond*dt*0.001
				delta = trigger:fillVehicle(self, delta, dt)
			end
			if delta <= 0 then
				self:setFillUnitIsFilling(false)
			end
		end
    end
    if self.isClient then
        for _, fillUnit in pairs(spec.fillUnits) do
            self:updateMeasurementNodes(fillUnit, dt, false)
        end
        self:updateAlarmTriggers(spec.activeAlarmTriggers)
        local needsUpdate = false
        -- stop effects
        for effect, time in pairs(spec.activeFillEffects) do
            time = time - dt
            if time < 0 then
                g_effectManager:stopEffects(effect)
                spec.activeFillEffects[effect] = nil
            else
                needsUpdate = true
                spec.activeFillEffects[effect] = time
            end
        end
        -- stop animations
        for animationNodes, time in pairs(spec.activeFillAnimations) do
            time = time - dt
            if time < 0 then
                g_animationManager:stopAnimations(animationNodes)
                spec.activeFillAnimations[animationNodes] = nil
            else
                needsUpdate = true
                spec.activeFillAnimations[animationNodes] = time
            end
        end
        if needsUpdate then
            self:raiseActive()
        end
    end
end


-----------------------------------------------------------------------------------	
-- Function to print debug values
-----------------------------------------------------------------------------------
function REAcog:PrintDebug(text)
	if REAcog.Debug then
		print(text);
	end;
end


if REAcog.ModActivated == nil then

	addModEventListener(REAcog);
	REAcog.ModActivated = true;
	REAcog.Debug = false;
	REAcog.FilePath = g_currentModDirectory;
	print("mod activated")

	-- Exchange standard GIANT'S functions for editet by REA
	REAcog.OriginalAddToPhysics = Vehicle.addToPhysics;
	Vehicle.addToPhysics = REAcog.addToPhysics;
	REAcog.OriginalloadComponentFromXML = Vehicle.loadComponentFromXML;
	Vehicle.loadComponentFromXML = REAcog.loadComponentFromXML;

	REAcog.OriginalupdateWheelBase = Wheels.updateWheelBase;
	Wheels.updateWheelBase = REAcog.updateWheelBase;
	Vehicle.updateMass = REAcog.updateMass;
	FillUnit.getAdditionalComponentMass = REAcog.getAdditionalComponentMass;
	FillUnit.onUpdateTick = REAcog.onUpdateTick;

	-- Standard functions exchanged
	print("New REA functions loaded")

end;
