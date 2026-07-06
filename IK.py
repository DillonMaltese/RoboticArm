import math

def inverse(xTarg, yTarg, zTarg, L1, L2, L3):
    #Find base angle
    baseTheta = math.atan2(yTarg, xTarg)
    
    #Horizontal reach 
    r = math.sqrt(xTarg**2 + yTarg**2)
    
    #Vertical reach
    z = zTarg + L3

    #Total reach
    D = math.sqrt(r**2 + z**2)
    
    cosElbow = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    cosElbow = max(-1.0, min(1.0, cosElbow))
    elbowTheta = math.acos(cosElbow)
    
    alpha = math.atan2(z, r)
    cosAlpha2 = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cosAlpha2 = max(-1.0, min(1.0, cosAlpha2))
    alpha2 = math.acos(cosAlpha2)
    shoulderTheta = alpha + alpha2
    
    armTipAngle = shoulderTheta + elbowTheta
    toolAngle = -math.pi / 2
    wristTheta = toolAngle - armTipAngle
    
    return baseTheta, shoulderTheta, elbowTheta, wristTheta
    
    
    
def forward(baseTheta, shoulderTheta, elbowTheta, wristTheta, L1, L2, L3):
    cosBase = math.cos(baseTheta)
    sinBase = math.sin(baseTheta)
    
    elbowX = L1 * math.cos(shoulderTheta) * cosBase
    elbowY = L1 * math.cos(shoulderTheta) * sinBase
    elbowZ = L1 * math.sin(shoulderTheta)
    
    #Cumulative arm angle at elbow = shoulderTheta + elbowTheta
    cumElbow = shoulderTheta + elbowTheta
    wristX = elbowX + L2 * math.cos(cumElbow) * cosBase
    wristY = elbowY + L2 * math.cos(cumElbow) * sinBase
    wristZ = elbowZ + L2 * math.sin(cumElbow)
    
    # Cumulative angle at wrist = shoulderTheta + elbowTheta + wristTheta
    cumWrist = shoulderTheta + elbowTheta + wristTheta
    tipX = wristX + L3 * math.cos(cumWrist) * cosBase
    tipY = wristY + L3 * math.cos(cumWrist) * sinBase
    tipZ = wristZ + L3 * math.sin(cumWrist)

    return tipX, tipY, tipZ

