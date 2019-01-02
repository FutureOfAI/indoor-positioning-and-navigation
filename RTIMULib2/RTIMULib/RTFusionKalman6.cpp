#include "RTFusionKalman6.h"
#include "RTIMUSettings.h"

RTFusionKalman6::RTFusionKalman6()
{
    reset();
}

RTFusionKalman6::~RTFusionKalman6()
{
}

void RTFusionKalman6::reset()
{

}

void RTFusionKalman6::predict()
{

	// Predict new state estimate Xkk_1 = Fk * Xk_1k_1
	m_state_dot = m_Fk * m_state + Eta;
}

void RTFusionKalman6::update()
{

}

void RTFusionKalman6::newIMUData(RTIMU_DATA& data, const RTIMUSettings *settings)
{

}
