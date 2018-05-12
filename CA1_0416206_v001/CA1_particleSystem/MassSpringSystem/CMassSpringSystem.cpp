#include <cmath>
#include <iostream>
#include "configFile.h"
#include "CMassSpringSystem.h"
#include "glut.h"
#include "Render_API.h"

#pragma comment( lib, "glut32.lib" )

const double g_cdDeltaT = 0.001f;
const double g_cdK	   = 2500.0f;
const double g_cdD	   = 50.0f;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Constructor & Destructor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CMassSpringSystem::CMassSpringSystem()
   :m_bDrawParticle(true),
    m_bDrawStruct(false),
    m_bDrawShear(false),
    m_bDrawBending(false),
    m_bSimulation(false),

    m_iIntegratorType(EXPLICIT_EULER),

    m_dDeltaT(g_cdDeltaT),
    m_dSpringCoefStruct(g_cdK),
    m_dSpringCoefShear(g_cdK),
    m_dSpringCoefBending(g_cdK),
    m_dDamperCoefStruct(g_cdD),
    m_dDamperCoefShear(g_cdD),
    m_dDamperCoefBending(g_cdD),
    m_dRotate(0.0f),

    m_Offset(Vector3d::ZERO),
    m_ForceField(Vector3d(0.0,-9.8,0.0)),

    m_GoalNet(),
    m_Balls()
{
}

CMassSpringSystem::CMassSpringSystem(const std::string &a_rcsConfigFilename)
:m_GoalNet(a_rcsConfigFilename)
{
    int iIntegratorType;
    double dSpringCoef,dDamperCoef,dHeightOffset;

    ConfigFile configFile;
    configFile.suppressWarnings(1);

    configFile.addOption("DrawParticle"        ,&m_bDrawParticle);
    configFile.addOption("DrawSpringStructural",&m_bDrawStruct);
    configFile.addOption("DrawSpringShear"     ,&m_bDrawShear);
    configFile.addOption("DrawSpringBending"   ,&m_bDrawBending);
    configFile.addOption("SimulationStart"     ,&m_bSimulation);

    configFile.addOption("IntegratorType",&iIntegratorType);
    configFile.addOption("DeltaT",&m_dDeltaT);
    configFile.addOption("SpringCoef",&dSpringCoef);
    configFile.addOption("DamperCoef",&dDamperCoef);
    configFile.addOption("Rotate",&m_dRotate);
    configFile.addOption("HeightOffset",&dHeightOffset);

    int code = configFile.parseOptions((char *)a_rcsConfigFilename.c_str());
    if(code == 1)
    {
        std::cout<<"Error in CMassSpringSystem constructor."<<std::endl;
        system("pause");
        exit(0);
    }
    m_iIntegratorType = CMassSpringSystem::EXPLICIT_EULER;
    if(iIntegratorType == 1)
    {
        m_iIntegratorType = CMassSpringSystem::RUNGE_KUTTA;
    }

    m_dSpringCoefStruct  = dSpringCoef;
    m_dSpringCoefShear   = dSpringCoef;
    m_dSpringCoefBending = dSpringCoef;
    m_dDamperCoefStruct  = dDamperCoef;
    m_dDamperCoefShear   = dDamperCoef;
    m_dDamperCoefBending = dDamperCoef;

    if(dHeightOffset<0.0 || dHeightOffset>10.0)
        dHeightOffset = 0.0;

    m_Offset       = Vector3d(0.0,dHeightOffset,0.0);
    m_ForceField   = Vector3d(0.0,-9.8,0.0);

    Reset();
}

CMassSpringSystem::CMassSpringSystem(const CMassSpringSystem &a_rcMassSpringSystem)
    :m_bDrawParticle(a_rcMassSpringSystem.m_bDrawParticle),
    m_bDrawStruct(a_rcMassSpringSystem.m_bDrawStruct),
    m_bDrawShear(a_rcMassSpringSystem.m_bDrawShear),
    m_bDrawBending(a_rcMassSpringSystem.m_bDrawBending),
    m_bSimulation(a_rcMassSpringSystem.m_bSimulation),

    m_iIntegratorType(a_rcMassSpringSystem.m_iIntegratorType),

    m_dDeltaT(a_rcMassSpringSystem.m_dDeltaT),
    m_dSpringCoefStruct(a_rcMassSpringSystem.m_dSpringCoefStruct),
    m_dSpringCoefShear(a_rcMassSpringSystem.m_dSpringCoefShear),
    m_dSpringCoefBending(a_rcMassSpringSystem.m_dSpringCoefBending),
    m_dDamperCoefStruct(a_rcMassSpringSystem.m_dDamperCoefStruct),
    m_dDamperCoefShear(a_rcMassSpringSystem.m_dDamperCoefShear),
    m_dDamperCoefBending(a_rcMassSpringSystem.m_dDamperCoefBending),
    m_dRotate(a_rcMassSpringSystem.m_dRotate),

    m_Offset(a_rcMassSpringSystem.m_Offset),
    m_ForceField(a_rcMassSpringSystem.m_ForceField)
{
}
CMassSpringSystem::~CMassSpringSystem()
{
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Draw
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::Draw()
{
    DrawGoalNet();
    DrawBall();
}

void CMassSpringSystem::DrawGoalNet()
{    
    // draw particle
    if (m_bDrawParticle)
    {
        glPushAttrib(GL_CURRENT_BIT);
        for (int uiI = 0; uiI < m_GoalNet.ParticleNum(); uiI++)
        {
            setColor3f(1.0, 0.0, 0.0);
            drawPoint(m_GoalNet.GetParticle(uiI).GetPosition(), 3.0);
        }
        glPopAttrib();
    }

    // draw spring
    glPushAttrib(GL_CURRENT_BIT);
    for (int uiI = 0; uiI < m_GoalNet.SpringNum(); uiI++)
    {
        if ((m_GoalNet.GetSpring(uiI).GetSpringType() == CSpring::Type_nStruct && m_bDrawStruct) ||
            (m_GoalNet.GetSpring(uiI).GetSpringType() == CSpring::Type_nShear && m_bDrawShear) ||
            (m_GoalNet.GetSpring(uiI).GetSpringType() == CSpring::Type_nBending && m_bDrawBending))
        {
            int iSpringStartID = m_GoalNet.GetSpring(uiI).GetSpringStartID();
            int iSpringEndID = m_GoalNet.GetSpring(uiI).GetSpringEndID();
            Vector3d springColor = m_GoalNet.GetSpring(uiI).GetSpringColor();
            Vector3d startPos = m_GoalNet.GetParticle(iSpringStartID).GetPosition();
            Vector3d endPos = m_GoalNet.GetParticle(iSpringEndID).GetPosition();

            setColor3fv(springColor);
            drawLine(startPos, endPos);                   
        }
    }
    glPopAttrib();

    // draw cylinder
    int widthNum = m_GoalNet.GetWidthNum();
    int heightNum = m_GoalNet.GetHeightNum();
    int lengthNum = m_GoalNet.GetLengthNum();

    int backBottomRightId = m_GoalNet.GetParticleID(0, 0, 0);
    int backBottomLeftId = m_GoalNet.GetParticleID(0, 0, lengthNum-1);
    int frontBottomRightId = m_GoalNet.GetParticleID(widthNum-1, 0, 0);
    int frontBottomLeftId = m_GoalNet.GetParticleID(widthNum-1, 0, lengthNum-1);
    int backTopRightId = m_GoalNet.GetParticleID(0, heightNum-1, 0);
    int backTopLeftId = m_GoalNet.GetParticleID(0, heightNum-1, lengthNum-1);
    int frontTopRightId = m_GoalNet.GetParticleID(widthNum-1, heightNum-1, 0);
    int frontTopLeftId = m_GoalNet.GetParticleID(widthNum-1, heightNum-1, lengthNum-1);

	// for vollyball baseket
	Vector3d touchFloor(0, -0.5, 0);
    /*
    drawCylinder(
        m_GoalNet.GetParticle(backBottomRightId).GetPosition(), 
        m_GoalNet.GetParticle(backBottomLeftId).GetPosition(), 
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backBottomRightId).GetPosition(),
        m_GoalNet.GetParticle(frontBottomRightId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backBottomLeftId).GetPosition(),
        m_GoalNet.GetParticle(frontBottomLeftId).GetPosition(),
        0.05);
    */
    drawCylinder(
		m_GoalNet.GetParticle(backBottomLeftId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(backTopLeftId).GetPosition(),
        0.05);
    drawCylinder(
		m_GoalNet.GetParticle(backBottomRightId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(backTopRightId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backTopRightId).GetPosition(),
        m_GoalNet.GetParticle(backTopLeftId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backTopRightId).GetPosition(),
        m_GoalNet.GetParticle(frontTopRightId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backTopLeftId).GetPosition(),
        m_GoalNet.GetParticle(frontTopLeftId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(frontTopRightId).GetPosition(),
        m_GoalNet.GetParticle(frontTopLeftId).GetPosition(),
        0.05);
	drawCylinder(
		m_GoalNet.GetParticle(frontBottomRightId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(frontTopRightId).GetPosition(),
        0.05);
    drawCylinder(
		m_GoalNet.GetParticle(frontBottomLeftId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(frontTopLeftId).GetPosition(),
        0.05);
}

void CMassSpringSystem::DrawBall()
{
    for (unsigned int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        drawSolidBall(m_Balls[ballIdx].GetPosition(), m_Balls[ballIdx].GetRadius());
    }
}


Vector3d CMassSpringSystem::CalcTriangleNormal(CParticle &p1,CParticle &p2,CParticle &p3)
{
    Vector3d pos1 = p1.GetPosition();
    Vector3d pos2 = p2.GetPosition();
    Vector3d pos3 = p3.GetPosition();

    Vector3d v1 = pos2-pos1;
    Vector3d v2 = pos3-pos1;

    return v1.CrossProduct(v2);
}

void CMassSpringSystem::DrawShadow(const Vector3d &a_rLightPos)
{
}
void CMassSpringSystem::DrawShadowPolygon(const Vector3d &a_rLightPos,const Vector3d &a_rNormalVec,
                                          const Vector3d &a_rVerPos1,const Vector3d &a_rVerPos2)
{
    Vector3d ShadowPos1,ShadowPos2,LightVec;
    LightVec = (a_rVerPos1 - a_rLightPos);
    LightVec.Normalize();
    ShadowPos1 = a_rVerPos1 + (a_rVerPos1 - a_rLightPos) * 5.0f;
    ShadowPos2 = a_rVerPos2 + (a_rVerPos2 - a_rLightPos) * 5.0f;
    
    if(a_rNormalVec.DotProduct(LightVec)<=0.0f)
    {
        glBegin( GL_QUADS );
        glVertex3dv( a_rVerPos1.val );
        glVertex3dv( ShadowPos1.val );
        glVertex3dv( ShadowPos2.val );
        glVertex3dv( a_rVerPos2.val );
        glEnd();
    }
    else
    {
        glBegin( GL_QUADS );
        glVertex3dv( a_rVerPos1.val );
        glVertex3dv( a_rVerPos2.val );
        glVertex3dv( ShadowPos2.val );
        glVertex3dv( ShadowPos1.val );
        glEnd();
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set and Update
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::Reset()
{ 
    m_GoalNet.Reset();
    m_Balls.clear();
}

void CMassSpringSystem::SetSpringCoef(const double a_cdSpringCoef, const CSpring::enType_t a_cSpringType)
{
    if (a_cSpringType == CSpring::Type_nStruct)
    {
        m_dSpringCoefStruct = a_cdSpringCoef;
        m_GoalNet.SetSpringCoef(a_cdSpringCoef, CSpring::Type_nStruct);
    }
    else if (a_cSpringType == CSpring::Type_nShear)
    {
        m_dSpringCoefShear = a_cdSpringCoef;
        m_GoalNet.SetSpringCoef(a_cdSpringCoef, CSpring::Type_nShear);
    }
    else if (a_cSpringType == CSpring::Type_nBending)
    {
        m_dSpringCoefBending = a_cdSpringCoef;
        m_GoalNet.SetSpringCoef(a_cdSpringCoef, CSpring::Type_nBending);
    }
    else
    {
        std::cout << "Error spring type in CMassSpringSystme SetSpringCoef" << std::endl;
    }

}

void CMassSpringSystem::SetDamperCoef(const double a_cdDamperCoef, const CSpring::enType_t a_cSpringType)
{
    if (a_cSpringType == CSpring::Type_nStruct)
    {
        m_dDamperCoefStruct = a_cdDamperCoef;
        m_GoalNet.SetDamperCoef(a_cdDamperCoef, CSpring::Type_nStruct);
    }
    else if (a_cSpringType == CSpring::Type_nShear)
    {
        m_dDamperCoefShear = a_cdDamperCoef;
        m_GoalNet.SetDamperCoef(a_cdDamperCoef, CSpring::Type_nShear);
    }
    else if (a_cSpringType == CSpring::Type_nBending)
    {
        m_dDamperCoefBending = a_cdDamperCoef;
        m_GoalNet.SetDamperCoef(a_cdDamperCoef, CSpring::Type_nBending);
    }
    else
    {
        std::cout << "Error spring type in CMassSpringSystme SetDamperCoef" << std::endl;
    }
}

double CMassSpringSystem::GetSpringCoef(const CSpring::enType_t a_cSpringType)
{
    if(a_cSpringType == CSpring::Type_nStruct)
    {
        return m_dSpringCoefStruct;
    }
    else if(a_cSpringType == CSpring::Type_nShear)
    {
        return m_dSpringCoefShear;
    }
    else if(a_cSpringType == CSpring::Type_nBending)
    {
        return m_dSpringCoefBending;
    }
    else
    {
        std::cout<<"Error spring type in CMassSpringSystme GetSpringCoef"<<std::endl;
        return -1.0;
    }
}
double CMassSpringSystem::GetDamperCoef(const CSpring::enType_t a_cSpringType)
{
    if(a_cSpringType == CSpring::Type_nStruct)
    {
        return m_dDamperCoefStruct;
    }
    else if(a_cSpringType == CSpring::Type_nShear)
    {
        return m_dDamperCoefShear;
    }
    else if(a_cSpringType == CSpring::Type_nBending)
    {
        return m_dDamperCoefBending;
    }
    else
    {
        std::cout<<"Error spring type in CMassSpringSystme GetDamperCoef"<<std::endl;
        return -1.0;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Simulation Part
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CMassSpringSystem::CheckStable()
{
    double threshold = 1e6;
    for(int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); pIdx++)
    {
        Vector3d particleVel = m_GoalNet.GetParticle(pIdx).GetVelocity();
        if (particleVel.Magnitude() > threshold)
        {
            return false;
        }  
    }
    return true;
}
void CMassSpringSystem::SimulationOneTimeStep()
{
    if(m_bSimulation)
    {
	    //ComputeAllForce();
        Integrate();
    }
    
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Initialization
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CMassSpringSystem::CreateBall()
{
    // randomly assign initial velocity and position to a ball
    Ball newBall;
	//Vector3d randomOffset((double)(rand() % 5 + 5.0), (double)(rand() % 5+5.0), (double)(rand() % 5));
	Vector3d randomOffset((double)(rand() % 5 + 5.0), (double)(rand() % 5 + 8.0), (double)(rand() % 5));
	Vector3d initBallPos = m_GoalNet.GetInitPos() + randomOffset;
    Vector3d initBallVel = (m_GoalNet.GetInitPos()- 2*initBallPos)*2.0;
    //Vector3d initBallVel = Vector3d::ZERO;
    newBall.SetPosition(initBallPos);
    newBall.SetVelocity(initBallVel);
    m_Balls.push_back(newBall);
}

int CMassSpringSystem::BallNum()
{
    return m_Balls.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Compute Force
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::ResetAllForce()
{
    for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); ++pIdx)
    {
        //TO DO 6
        m_GoalNet.GetParticle(pIdx).SetAcceleration(Vector3d::ZERO);
    }

    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        m_Balls[ballIdx].SetAcceleration(Vector3d::ZERO);
    }
}

void CMassSpringSystem::ComputeAllForce()
{
    ComputeParticleForce();
    ComputeBallForce();
}

void CMassSpringSystem::ComputeParticleForce()
{
    m_GoalNet.AddForceField(m_ForceField);
    m_GoalNet.ComputeInternalForce();
}

void CMassSpringSystem::ComputeBallForce()
{
    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        //m_Balls[ballIdx].SetAcceleration(m_ForceField);
        m_Balls[ballIdx].AddForce(m_ForceField * m_Balls[ballIdx].GetMass());
    }
}

void CMassSpringSystem::HandleCollision()
{
    NetPlaneCollision();
    BallPlaneCollision();
    BallToBallCollision();
    BallNetCollision();
}

void CMassSpringSystem::NetPlaneCollision()
{
    //TO DO 2
    static const double eEPSILON = 0.01;
    double resistCoef = 0.5;
    double frictionCoef = 0.3;
	Vector3d Plane(0.0, -1.0, 0.0);
	Vector3d PlaneN(0.0, 1.0, 0.0);
	for (int Id = 0; Id < m_GoalNet.ParticleNum(); Id++)
	{
		if (PlaneN.DotProduct(m_GoalNet.GetParticle(Id).GetPosition() - Plane) < eEPSILON && PlaneN.DotProduct(m_GoalNet.GetParticle(Id).GetVelocity()) < 0)
		{
			if (PlaneN.DotProduct(m_GoalNet.GetParticle(Id).GetForce()) < 0)
			{
				Vector3d Fc = -1 * (PlaneN.DotProduct(m_GoalNet.GetParticle(Id).GetForce())) * PlaneN;
				Vector3d proj = m_GoalNet.GetParticle(Id).GetVelocity() * PlaneN;
				Vector3d Vt = m_GoalNet.GetParticle(Id).GetVelocity() - proj;
				Vector3d friction = -1 * frictionCoef * (-1 * PlaneN.DotProduct(m_GoalNet.GetParticle(Id).GetForce())) * Vt;
				m_GoalNet.GetParticle(Id).AddForce(Fc);
				m_GoalNet.GetParticle(Id).AddForce(friction);
			}
			Vector3d ReversedVelocity(m_GoalNet.GetParticle(Id).GetVelocity().val[0], -1 * resistCoef * m_GoalNet.GetParticle(Id).GetVelocity().val[1], m_GoalNet.GetParticle(Id).GetVelocity().val[2]);
			m_GoalNet.GetParticle(Id).SetVelocity(ReversedVelocity);
		}
	}
}

void CMassSpringSystem::BallPlaneCollision()
{
    //TO DO 2
    static const double eEPSILON = 0.01;
    double resistCoef = 0.5;
    double frictionCoef = 0.3;
	Vector3d Plane(0.0, -1.0, 0.0);
	Vector3d PlaneN(0.0, 1.0, 0.0);
	for (int i = 0; i < m_Balls.size(); i++)
	{
		if (PlaneN.DotProduct(m_Balls[i].GetPosition() - Plane) < eEPSILON + m_Balls[i].GetRadius() && PlaneN.DotProduct(m_Balls[i].GetVelocity()) < 0)
		{
			if (PlaneN.DotProduct(m_Balls[i].GetForce()) < 0)
			{
				Vector3d Fc = -1 * (PlaneN.DotProduct(m_Balls[i].GetForce())) * PlaneN;
				Vector3d proj = m_Balls[i].GetVelocity() * PlaneN;
				Vector3d Vt = m_Balls[i].GetVelocity() + proj;
				Vector3d friction = -1 * frictionCoef * (-1 * PlaneN.DotProduct(m_Balls[i].GetForce())) * Vt;
			}
			Vector3d ReversedVelocity(m_Balls[i].GetVelocity().val[0], -1 * resistCoef * m_Balls[i].GetVelocity().val[1], m_Balls[i].GetVelocity().val[2]);
			m_Balls[i].SetVelocity(ReversedVelocity);
		}
	}
}

void CMassSpringSystem::BallToBallCollision()
{
    static const double eEPSILON = 0.01;
	//TO DO 2
	for (int i = 0; i < m_Balls.size(); i++)
	{
		for (int j = 0; j < m_Balls.size(); j++)
		{
			if ((m_Balls[i].GetPosition() - m_Balls[j].GetPosition()).DotProduct(m_Balls[i].GetVelocity() - m_Balls[j].GetVelocity()) < 0)
			{
				if (i != j && (m_Balls[i].GetRadius() + m_Balls[j].GetRadius()) >= (m_Balls[i].GetPosition() - m_Balls[j].GetPosition()).Length())
				{
					Vector3d dir = (m_Balls[j].GetPosition() - m_Balls[i].GetPosition()).NormalizedCopy();
					Vector3d v1n = dir * (m_Balls[i].GetVelocity().DotProduct(dir));
					Vector3d v1t = m_Balls[i].GetVelocity() - v1n;
					Vector3d dir2 = (m_Balls[i].GetPosition() - m_Balls[j].GetPosition()).NormalizedCopy();
					Vector3d v2n = dir2 * (m_Balls[j].GetVelocity().DotProduct(dir2));
					Vector3d v2t = m_Balls[j].GetVelocity() - v2n;
					Vector3d v1 = (v1n * (m_Balls[i].GetMass() - m_Balls[j].GetMass()) + 2 * m_Balls[j].GetMass() * v2n) / (m_Balls[i].GetMass() + m_Balls[j].GetMass()) + v1t;
					Vector3d v2 = (v2n * (m_Balls[j].GetMass() - m_Balls[i].GetMass()) + 2 * m_Balls[i].GetMass() * v1n) / (m_Balls[i].GetMass() + m_Balls[j].GetMass()) + v2t;
					m_Balls[i].SetVelocity(v1);
					m_Balls[j].SetVelocity(v2);
				}
			}
			
		}
	}
}

void CMassSpringSystem::BallNetCollision()
{
    static const double eEPSILON = 0.01;
	//TO DO 2
	for (int i = 0; i < m_Balls.size(); i++)
	{
		for (int j = 0; j < m_GoalNet.ParticleNum(); j++)
		{
			if ((m_Balls[i].GetRadius() + eEPSILON) >= (m_Balls[i].GetPosition() - m_GoalNet.GetParticle(j).GetPosition()).Length())
			{ 
				if (m_GoalNet.GetParticle(j).IsMovable() == false && (m_Balls[i].GetPosition() - m_GoalNet.GetParticle(j).GetPosition()).DotProduct(m_Balls[i].GetVelocity() - m_GoalNet.GetParticle(j).GetVelocity()) < 0)
				{
					Vector3d Nv = -0.7 * m_Balls[i].GetVelocity();
					m_Balls[i].SetVelocity(Nv);
				}
				else
				{
					Vector3d dir = (m_GoalNet.GetParticle(j).GetPosition() - m_Balls[i].GetPosition()).NormalizedCopy();
					Vector3d v1n = dir * (m_Balls[i].GetVelocity().DotProduct(dir));
					Vector3d v1t = m_Balls[i].GetVelocity() - v1n;
					Vector3d dir2 = (m_Balls[i].GetPosition() - m_GoalNet.GetParticle(j).GetPosition()).NormalizedCopy();
					Vector3d v2n = dir2 * (m_GoalNet.GetParticle(j).GetVelocity().DotProduct(dir2));
					Vector3d v2t = m_GoalNet.GetParticle(j).GetVelocity() - v2n;
					Vector3d v1 = (v1n * (m_Balls[i].GetMass() - m_GoalNet.GetParticle(j).GetMass()) + 2 * m_GoalNet.GetParticle(j).GetMass() * v2n) / (m_Balls[i].GetMass() + m_GoalNet.GetParticle(j).GetMass()) + v1t;
					Vector3d v2 = (v2n * (m_GoalNet.GetParticle(j).GetMass() - m_Balls[i].GetMass()) + 2 * m_Balls[i].GetMass() * v1n) / (m_Balls[i].GetMass() + m_GoalNet.GetParticle(j).GetMass()) + v2t;
					m_Balls[i].SetVelocity(v1);
					m_GoalNet.GetParticle(j).SetVelocity(v2);
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Integrator
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::Integrate()
{
    ResetContactForce();

    if(m_iIntegratorType == CMassSpringSystem::EXPLICIT_EULER)
    {
        ComputeAllForce();
        HandleCollision();
	    ExplicitEuler();
        ResetAllForce();
    }
    else if(m_iIntegratorType == CMassSpringSystem::RUNGE_KUTTA)
    {
        RungeKutta();
        ResetAllForce();
    }
    else
    {
        std::cout<<"Error integrator type, use explicit Euler instead!!"<<std::endl;
        ComputeAllForce();
        HandleCollision();
        ExplicitEuler();
        ResetAllForce();
    }
}

void CMassSpringSystem::ExplicitEuler()
{
    // Goal Net
    for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); ++pIdx)
    {
        //TO DO 6
		double deltaT = 0.001;
		m_GoalNet.GetParticle(pIdx).AddVelocity(m_GoalNet.GetParticle(pIdx).GetForce() / m_GoalNet.GetParticle(pIdx).GetMass() * deltaT);
		m_GoalNet.GetParticle(pIdx).AddPosition(m_GoalNet.GetParticle(pIdx).GetVelocity() * deltaT);
    }

    // Balls
    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
		//TO DO 6
		double deltaT = 0.001;
		m_Balls[ballIdx].AddVelocity(m_Balls[ballIdx].GetForce() / m_Balls[ballIdx].GetMass() * deltaT);
		m_Balls[ballIdx].AddPosition(m_Balls[ballIdx].GetVelocity() * deltaT);
    }
}

void CMassSpringSystem::RungeKutta()
{
    //TO DO 7
	ComputeAllForce();
	NetPlaneCollision();
	BallPlaneCollision();
    BallNetCollision();
    BallToBallCollision();
    ParticleRungeKutta();
    BallRungeKutta();
}

void CMassSpringSystem::ParticleRungeKutta()
{
	//TO DO 7
    struct StateStep
    {
        Vector3d deltaVel;
        Vector3d deltaPos;
    };

    //container to store k1~k4 for each particles
    vector<Vector3d> curPosCntr, curVelCntr;
    vector<StateStep> k1StepCntr, k2StepCntr, k3StepCntr, k4StepCntr;
	curPosCntr.resize(m_GoalNet.ParticleNum());
	curVelCntr.resize(m_GoalNet.ParticleNum());
	k1StepCntr.resize(m_GoalNet.ParticleNum());
	k2StepCntr.resize(m_GoalNet.ParticleNum());
	k3StepCntr.resize(m_GoalNet.ParticleNum());
	k4StepCntr.resize(m_GoalNet.ParticleNum());
	for (int i = 0; i < m_GoalNet.ParticleNum(); i++)
	{
		k1StepCntr[i].deltaPos = m_GoalNet.GetParticle(i).GetVelocity() * g_cdDeltaT;
		k1StepCntr[i].deltaVel = m_GoalNet.GetParticle(i).GetVelocity();
		k2StepCntr[i].deltaPos = (m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT / 2) * g_cdDeltaT;
		k2StepCntr[i].deltaVel = m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT;
		k3StepCntr[i].deltaPos = (m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT / 2) * g_cdDeltaT;
		k3StepCntr[i].deltaVel = m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT;
		k4StepCntr[i].deltaPos = (m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT) * g_cdDeltaT;
		k4StepCntr[i].deltaVel = m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT;
		Vector3d slope = (k1StepCntr[i].deltaPos + 2 * k2StepCntr[i].deltaPos + 2 * k3StepCntr[i].deltaPos + k4StepCntr[i].deltaPos) / 6;
		Vector3d NewVelocity = m_GoalNet.GetParticle(i).GetVelocity() + m_GoalNet.GetParticle(i).GetAcceleration() * g_cdDeltaT;
		m_GoalNet.GetParticle(i).AddPosition(slope);
		m_GoalNet.GetParticle(i).SetVelocity(NewVelocity);
	}
}

void CMassSpringSystem::BallRungeKutta()
{
	//TO DO 7
    struct StateStep
    {
        Vector3d deltaVel;
        Vector3d deltaPos;
    };

    //container to store k1~k4 for each particles
    vector<Vector3d> curPosCntr, curVelCntr;
    vector<StateStep> k1StepCntr, k2StepCntr, k3StepCntr, k4StepCntr;
	curPosCntr.resize(m_Balls.size());
	curVelCntr.resize(m_Balls.size());
	k1StepCntr.resize(m_Balls.size());
	k2StepCntr.resize(m_Balls.size());
	k3StepCntr.resize(m_Balls.size());
	k4StepCntr.resize(m_Balls.size());
	for (int i = 0; i < m_Balls.size(); i++)
	{
		k1StepCntr[i].deltaPos = m_Balls[i].GetVelocity() * g_cdDeltaT;
		k1StepCntr[i].deltaVel = m_Balls[i].GetVelocity();
		k2StepCntr[i].deltaPos = (m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT / 2) * g_cdDeltaT;
		k2StepCntr[i].deltaVel = m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT;
		k3StepCntr[i].deltaPos = (m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT / 2) * g_cdDeltaT;
		k3StepCntr[i].deltaVel = m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT;
		k4StepCntr[i].deltaPos = (m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT) * g_cdDeltaT;
		k4StepCntr[i].deltaVel = m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT;
		Vector3d slope = (k1StepCntr[i].deltaPos + 2 * k2StepCntr[i].deltaPos + 2 * k3StepCntr[i].deltaPos + k4StepCntr[i].deltaPos) / 6;
		Vector3d NewVelocity = m_Balls[i].GetVelocity() + m_Balls[i].GetAcceleration() * g_cdDeltaT;
		m_Balls[i].AddPosition(slope);
		m_Balls[i].SetVelocity(NewVelocity);
	}
}

void CMassSpringSystem::ResetContactForce()
{
    m_BallContactForce.clear();
    m_BallContactForce.resize(BallNum());
    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        m_BallContactForce.push_back(Vector3d::ZERO);
    }
    m_ParticleContactForce.clear();
    m_ParticleContactForce.resize(m_GoalNet.ParticleNum());
    for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); ++pIdx)
    {
        m_ParticleContactForce.push_back(Vector3d::ZERO);
    }
}