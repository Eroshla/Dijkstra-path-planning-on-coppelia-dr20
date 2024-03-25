--lua

sim=require'sim'

function sysCall_init() 
    robot=sim.getObject('.')
    leftJoint=sim.getObject("./leftWheelJoint_")
    rightJoint=sim.getObject("./rightWheelJoint_")

    -- Inicialize a matriz 8x8
    matrizXY = {}
    for i=1,8 do
        matrizXY[i] = {}
        for j=1,8 do
            local dummyNumber = (i - 1) * 8 + j
            local dummy = sim.getObjectHandle(tostring(dummyNumber))
            local position = sim.getObjectPosition(dummy, -1)
            matrizXY[i][j] = {x = position[1], y = position[2]}
        end
    end

    matrizCaminho = {}
    for i=1,8 do
        matrizCaminho[i] = {}
        for j=1,8 do
            matrizCaminho[i][j] = 999
        end
    end

    -- Inicialize a tabela de cuboids
    cuboids = {}
    for i = 1, 4 do
        cuboid = sim.getObjectHandle('Cuboid' .. tostring(i))
        table.insert(cuboids, cuboid)
    end



    function compareCuboidPositionWithDummies()
        -- Compare cada cuboid position com dummies
        for index = 1, #cuboids do
            local cuboidPosition = sim.getObjectPosition(cuboids[index], -1)
    
            for i=1,8 do
                for j=1,8 do
                    local dummyPosition = matrizXY[i][j]
                    if math.abs(cuboidPosition[1] - dummyPosition.x) < 0.2 and math.abs(cuboidPosition[2] - dummyPosition.y) < 0.2 then
                        matrizCaminho[i][j] = 80  
                    end
                end
            end
        end
    end
    
    function compareRobotPositionWithDummies()
        local robotPosition = sim.getObjectPosition(robot, -1)

        for i=1,8 do
            for j=1,8 do
                local dummyPosition = matrizXY[i][j]
                if math.abs(robotPosition[1] - dummyPosition.x) < 0.4 and math.abs(robotPosition[2] - dummyPosition.y) < 0.4 then
                    matrizCaminho[i][j] = 0
                end
            end
        end
    end

    function compareGoalPositionWithDummies()
        local goal = sim.getObjectHandle('Goal')
        local goalPosition = sim.getObjectPosition(goal, -1)
    
        for i=1,8 do
            for j=1,8 do
                local dummyPosition = matrizXY[i][j]
                if math.abs(goalPosition[1] - dummyPosition.x) < 0.4 and math.abs(goalPosition[2] - dummyPosition.y) < 0.4 then
                    matrizCaminho[i][j] = 20
                end
            end
        end
    end

    compareRobotPositionWithDummies()
    compareCuboidPositionWithDummies()
    compareGoalPositionWithDummies()

    
    function drawPath(path)
        local black = {0, 0, 0}  -- RGB para preto
        local lineSize = 3  -- Tamanho da linha
        local discSize = 3  -- Tamanho do disco
        local lineContainer = sim.addDrawingObject(sim.drawing_lines, lineSize, 0, -1, 5000, black)
        local discContainer = sim.addDrawingObject(sim.drawing_discpoints, discSize, 0, -1, 1, black)
    
        for i = 1, #path - 1 do
            local point1 = matrizXY[path[i].x][path[i].y]
            local point2 = matrizXY[path[i + 1].x][path[i + 1].y]
            -- Desenha uma linha do ponto1 ao ponto2
            sim.addDrawingObjectItem(lineContainer, {point1.x, point1.y, 0, point2.x, point2.y, 0})
        end
    
        -- Desenha um disco no ponto final
        local finalPoint = matrizXY[path[#path-1].x][path[#path-1].y]
        sim.addDrawingObjectItem(discContainer, {finalPoint.x, finalPoint.y, 0})
    end
    
    function calculateCosts(matrizCaminho)
        local start, goal = {x=0, y=0}, {x=0, y=0}
        local visited, distance, previous = {}, {}, {}
        local dx = {0, 0, -1, 1}
        local dy = {-1, 1, 0, 0}

        for i=1,#matrizCaminho do
            visited[i] = {}
            distance[i] = {}
            previous[i] = {}
            for j=1,#matrizCaminho[i] do
                visited[i][j] = false
                distance[i][j] = math.huge
                previous[i][j] = nil
                if matrizCaminho[i][j] == 0 then
                    start = {x=i, y=j}
                elseif matrizCaminho[i][j] == 20 then
                    goal = {x=i, y=j}
                end
            end
        end

        local queue = {{x=start.x, y=start.y, dist=0}}
        distance[start.x][start.y] = 0

        while #queue > 0 do
            table.sort(queue, function(a, b) return a.dist < b.dist end)
            local current = table.remove(queue, 1)
            visited[current.x][current.y] = true

            for i=1,4 do
                local nx, ny = current.x + dx[i], current.y + dy[i]
                if nx >= 1 and nx <= #visited and ny >= 1 and ny <= #visited[1] and not visited[nx][ny] and matrizCaminho[nx][ny] ~= 80 then
                    local newDist = current.dist + 1
                    if newDist < distance[nx][ny] then
                        distance[nx][ny] = newDist
                        previous[nx][ny] = current
                        table.insert(queue, {x=nx, y=ny, dist=newDist})
                    end
                end
            end
        end

        local path = {}
        local current = goal
        while current do
            table.insert(path, 1, current)
            current = previous[current.x][current.y]
        end

        drawPath(path)  -- Adicione esta linha

        for i, point in ipairs(path) do
            print(string.format("Point %d: (%d, %d)", i, point.x, point.y))
        end

        return distance[goal.x][goal.y], path
    end

    cost , path = calculateCosts(matrizCaminho)

    function savePathFollow(path)
        pathFollow = {}
        for i, point in ipairs(path) do
            local dummyPosition = matrizXY[point.x][point.y]
            table.insert(pathFollow, {x = dummyPosition.x, y = dummyPosition.y})
        end

        return pathFollow

    end


end

function sysCall_cleanup() 
    print(matrizCaminho)
end 

function sysCall_thread()
    s=sim.getObjectSizeFactor(robot) -- make sure that if we scale the robot during simulation, other values are scaled too!
    v0=0.6*s
    wheelDiameter=0.085*s
    interWheelDistance=0.254*s

    pathFollow = savePathFollow(path)

    print(pathFollow)
-- Initialize PID controller
local Kp = 2.0  -- Proportional gain
local Ki = 0.1  -- Integral gain
local Kd = 0.1  -- Derivative gain
local integral = 0.0
local previousError = 0.0
local errorThreshold = 0.25  -- Threshold for moving forward

for i, point in ipairs(pathFollow) do
    local targetPosition = {point.x, point.y, 0}
    local distanceThreshold = 0.2
    local angleThreshold = 2.5 -- Modify the angle threshold to be 3

    while true do
        local currentPosition = sim.getObjectPosition(robot, -1)
        local currentOrientation = sim.getObjectOrientation(robot, -1)
        if currentOrientation[3] < 0 then
            currentOrientation[3] = currentOrientation[3] + 2*math.pi  -- Normalize to 0 to 2pi
        end

        local distance = math.sqrt((targetPosition[1] - currentPosition[1])^2 + (targetPosition[2] - currentPosition[2])^2)
        local angle = math.atan2(targetPosition[2] - currentPosition[2], targetPosition[1] - currentPosition[1])
        if angle < 0 then
            angle = angle + 2*math.pi  -- Normalize to 0 to 2pi
        end

        -- PID controller
        local error = angle - currentOrientation[3]
        integral = integral + error
        local derivative = error - previousError
        local output = Kp*error + Ki*integral + Kd*derivative
        previousError = error

        if distance < distanceThreshold and math.abs(angle) < angleThreshold then
            break  -- Reached the target point with correct angle
        end

        if math.abs(error) < errorThreshold then
            -- Move forward
            local movementSpeed = 3.5  -- Increase the speed
            sim.setJointTargetVelocity(leftJoint, movementSpeed)
            sim.setJointTargetVelocity(rightJoint, movementSpeed)
        else
            -- Rotate towards the target point
            local rotationSpeed = output * 0.5  -- Increase the speed
            if error > 0 then
                -- Rotate left
                sim.setJointTargetVelocity(leftJoint, -rotationSpeed)
                sim.setJointTargetVelocity(rightJoint, rotationSpeed)
            else
                -- Rotate right
                sim.setJointTargetVelocity(leftJoint, rotationSpeed)
                sim.setJointTargetVelocity(rightJoint, -rotationSpeed)
            end
        end

        sim.switchThread()  -- Allow other threads to run
    end

    -- Stop the robot
    sim.setJointTargetVelocity(leftJoint, 0)
    sim.setJointTargetVelocity(rightJoint, 0)

    sim.switchThread()  -- Allow other threads to run
end
end