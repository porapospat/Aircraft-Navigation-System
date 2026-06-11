/**
 * @file    ahrsimu.c
 * @brief   Реализация алгоритма AHRS/IMU на основе фильтра Madgwick
 */

/* Includes ------------------------------------------------------------------*/
#include "ahrsimu.h"

/* Private function prototypes -----------------------------------------------*/
/* Нормализатор вектора: возвращает 1/sqrt(x) (используется для нормализации акселерометра и кватерниона) */
static float oneOverSqrt(float x);
/* Переводит кватернион в углы Эйлера (Roll/Pitch/Yaw) в градусах */
static void calculateAngles(AHRSIMU_t* AHRSIMU);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 				Инициализация AHRS/IMU фильтра
 * @param  AHRSIMU      Указатель на структуру AHRSIMU_t
 * @param  sampleRate   Частота обновления 
 * @param  beta         Коэффициент beta фильтра Madgwick
 * @param  inclination  Магнитное склонение 
 */
void AHRSIMU_Init(AHRSIMU_t* AHRSIMU, float sampleRate, float beta, float inclination) 
{
	/* Параметры фильтра и начальное состояние */
	AHRSIMU->_beta = beta;           	   /* Коэффициент коррекции акселерометра */
	AHRSIMU->_sampleRate = 1 / sampleRate; /* Период дискретизации dt = 1/Fs */
	AHRSIMU->Inclination = inclination;    /* Магнитное склонение (для коррекции Yaw) */
	
	/* Единичный кватернион: отсутствие вращения */
	AHRSIMU->_q0 = 1.0f;
	AHRSIMU->_q1 = 0.0f;
	AHRSIMU->_q2 = 0.0f;
	AHRSIMU->_q3 = 0.0f;
}

#if 0
/**
 * @brief  Обновление AHRS с данными акселерометра, гироскопа и магнитометра
 * @param  AHRSIMU  Указатель на структуру AHRSIMU_t
 * @param  gx       Угловая скорость X (рад/с)
 * @param  gy       Угловая скорость Y (рад/с)
 * @param  gz       Угловая скорость Z (рад/с)
 * @param  ax       Ускорение X (g)
 * @param  ay       Ускорение Y (g)
 * @param  az       Ускорение Z (g)
 * @param  mx       Магнитное поле X
 * @param  my       Магнитное поле Y
 * @param  mz       Магнитное поле Z
 */
void AHRSIMU_UpdateAHRS(AHRSIMU_t* AHRSIMU, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	if ((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)) 
	{        
			qDot1 = 0.5f * (-AHRSIMU->_q1 * gx - AHRSIMU->_q2 * gy - AHRSIMU->_q3 * gz);
			qDot2 = 0.5f * (AHRSIMU->_q0 * gx + AHRSIMU->_q2 * gz - AHRSIMU->_q3 * gy);
			qDot3 = 0.5f * (AHRSIMU->_q0 * gy - AHRSIMU->_q1 * gz + AHRSIMU->_q3 * gx);
			qDot4 = 0.5f * (AHRSIMU->_q0 * gz + AHRSIMU->_q1 * gy - AHRSIMU->_q2 * gx);

			if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

					recipNorm = oneOverSqrt(ax * ax + ay * ay + az * az);
					ax *= recipNorm;
					ay *= recipNorm;
					az *= recipNorm;

					recipNorm = oneOverSqrt(mx * mx + my * my + mz * mz);
					mx *= recipNorm;
					my *= recipNorm;
					mz *= recipNorm;

					_2q0mx = 2.0f * AHRSIMU->_q0 * mx;
					_2q0my = 2.0f * AHRSIMU->_q0 * my;
					_2q0mz = 2.0f * AHRSIMU->_q0 * mz;
					_2q1mx = 2.0f * AHRSIMU->_q1 * mx;
					_2q0 = 2.0f * AHRSIMU->_q0;
					_2q1 = 2.0f * AHRSIMU->_q1;
					_2q2 = 2.0f * AHRSIMU->_q2;
					_2q3 = 2.0f * AHRSIMU->_q3;
					_2q0q2 = 2.0f * AHRSIMU->_q0 * AHRSIMU->_q2;
					_2q2q3 = 2.0f * AHRSIMU->_q2 * AHRSIMU->_q3;
					q0q0 = AHRSIMU->_q0 * AHRSIMU->_q0;
					q0q1 = AHRSIMU->_q0 * AHRSIMU->_q1;
					q0q2 = AHRSIMU->_q0 * AHRSIMU->_q2;
					q0q3 = AHRSIMU->_q0 * AHRSIMU->_q3;
					q1q1 = AHRSIMU->_q1 * AHRSIMU->_q1;
					q1q2 = AHRSIMU->_q1 * AHRSIMU->_q2;
					q1q3 = AHRSIMU->_q1 * AHRSIMU->_q3;
					q2q2 = AHRSIMU->_q2 * AHRSIMU->_q2;
					q2q3 = AHRSIMU->_q2 * AHRSIMU->_q3;
					q3q3 = AHRSIMU->_q3 * AHRSIMU->_q3;

					hx = mx * q0q0 - _2q0my * AHRSIMU->_q3 + _2q0mz * AHRSIMU->_q2 + mx * q1q1 + _2q1 * my * AHRSIMU->_q2 + _2q1 * mz * AHRSIMU->_q3 - mx * q2q2 - mx * q3q3;
					hy = _2q0mx * AHRSIMU->_q3 + my * q0q0 - _2q0mz * AHRSIMU->_q1 + _2q1mx * AHRSIMU->_q2 - my * q1q1 + my * q2q2 + _2q2 * mz * AHRSIMU->_q3 - my * q3q3;
					_2bx = (float) sqrt(hx * hx + hy * hy);
					_2bz = -_2q0mx * AHRSIMU->_q2 + _2q0my * AHRSIMU->_q1 + mz * q0q0 + _2q1mx * AHRSIMU->_q3 - mz * q1q1 + _2q2 * my * AHRSIMU->_q3 - mz * q2q2 + mz * q3q3;
					_2bx *= 2.0f;
					_2bz *= 2.0f;
					_4bx = 2.0f * _2bx;
					_4bz = 2.0f * _2bz;

					s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * AHRSIMU->_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * AHRSIMU->_q3 + _2bz * AHRSIMU->_q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * AHRSIMU->_q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * AHRSIMU->_q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * AHRSIMU->_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * AHRSIMU->_q2 + _2bz * AHRSIMU->_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * AHRSIMU->_q3 - _4bz * AHRSIMU->_q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * AHRSIMU->_q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * AHRSIMU->_q2 - _2bz * AHRSIMU->_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * AHRSIMU->_q1 + _2bz * AHRSIMU->_q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * AHRSIMU->_q0 - _4bz * AHRSIMU->_q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * AHRSIMU->_q3 + _2bz * AHRSIMU->_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * AHRSIMU->_q0 + _2bz * AHRSIMU->_q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * AHRSIMU->_q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					
					recipNorm = oneOverSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
					s0 *= recipNorm;
					s1 *= recipNorm;
					s2 *= recipNorm;
					s3 *= recipNorm;

					qDot1 -= AHRSIMU->_beta * s0;
					qDot2 -= AHRSIMU->_beta * s1;
					qDot3 -= AHRSIMU->_beta * s2;
					qDot4 -= AHRSIMU->_beta * s3;
			}

			AHRSIMU->_q0 += qDot1 * AHRSIMU->_sampleRate;
			AHRSIMU->_q1 += qDot2 * AHRSIMU->_sampleRate;
			AHRSIMU->_q2 += qDot3 * AHRSIMU->_sampleRate;
			AHRSIMU->_q3 += qDot4 * AHRSIMU->_sampleRate;

			recipNorm = oneOverSqrt(AHRSIMU->_q0 * AHRSIMU->_q0 + AHRSIMU->_q1 * AHRSIMU->_q1 + AHRSIMU->_q2 * AHRSIMU->_q2 + AHRSIMU->_q3 * AHRSIMU->_q3);
			AHRSIMU->_q0 *= recipNorm;
			AHRSIMU->_q1 *= recipNorm;
			AHRSIMU->_q2 *= recipNorm;
			AHRSIMU->_q3 *= recipNorm;
	} 
	else 
	{
			AHRSIMU_UpdateIMU(AHRSIMU, gx, gy, gz, ax, ay, az);
	}
	
	calculateAngles(AHRSIMU);
}
#endif

/**
 * @brief  			Обновление IMU (без магнитометра) — градиентный спуск Madgwick
 * @param  AHRSIMU  Указатель на структуру AHRSIMU_t
 * @param  gx       Угловая скорость X 
 * @param  gy       Угловая скорость Y 
 * @param  gz       Угловая скорость Z 
 * @param  ax       Ускорение X 
 * @param  ay       Ускорение Y 
 * @param  az       Ускорение Z 
 */
void AHRSIMU_UpdateIMU(AHRSIMU_t* AHRSIMU, float gx, float gy, float gz, float ax, float ay, float az) 
{
	float recipNorm;
	/* Вектор градиента (ошибка между ожидаемым и измеренным направлением g) */
	float s0, s1, s2, s3;
	/* Производные кватерниона по времени (без и с учётом обратной связи) */
	float qDot1, qDot2, qDot3, qDot4;
	/* Вспомогательные величины для ускорения вычислений */
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	/* 1. Производные кватерниона только от гироскопа (чистое интегрирование) */
	qDot1 = 0.5f * (-AHRSIMU->_q1 * gx - AHRSIMU->_q2 * gy - AHRSIMU->_q3 * gz);
	qDot2 = 0.5f * (AHRSIMU->_q0 * gx + AHRSIMU->_q2 * gz - AHRSIMU->_q3 * gy);
	qDot3 = 0.5f * (AHRSIMU->_q0 * gy - AHRSIMU->_q1 * gz + AHRSIMU->_q3 * gx);
	qDot4 = 0.5f * (AHRSIMU->_q0 * gz + AHRSIMU->_q1 * gy - AHRSIMU->_q2 * gx);

	if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) 
	{
		/* 2. Нормализация вектора ускорения */
		recipNorm = oneOverSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

		/* 3. Вычисление вспомогательных переменных для ускорения */
		_2q0 = 2.0f * AHRSIMU->_q0;
		_2q1 = 2.0f * AHRSIMU->_q1;
		_2q2 = 2.0f * AHRSIMU->_q2;
		_2q3 = 2.0f * AHRSIMU->_q3;
		_4q0 = 4.0f * AHRSIMU->_q0;
		_4q1 = 4.0f * AHRSIMU->_q1;
		_4q2 = 4.0f * AHRSIMU->_q2;
		_8q1 = 8.0f * AHRSIMU->_q1;
		_8q2 = 8.0f * AHRSIMU->_q2;
		q0q0 = AHRSIMU->_q0 * AHRSIMU->_q0;
		q1q1 = AHRSIMU->_q1 * AHRSIMU->_q1;
		q2q2 = AHRSIMU->_q2 * AHRSIMU->_q2;
		q3q3 = AHRSIMU->_q3 * AHRSIMU->_q3;

		/* 4. Градиент функции ошибки (ориентация кватерниона относительно направления g) */
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * AHRSIMU->_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * AHRSIMU->_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * AHRSIMU->_q3 - _2q1 * ax + 4.0f * q2q2 * AHRSIMU->_q3 - _2q2 * ay;
		
		/* 5. Нормализация вектора градиента, чтобы масштаб beta имел однозначный смысл */
		recipNorm = oneOverSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
		s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

		/* 6. Коррекция производных кватерниона (обратная связь по акселерометру) */
		qDot1 -= AHRSIMU->_beta * s0;
		qDot2 -= AHRSIMU->_beta * s1;
		qDot3 -= AHRSIMU->_beta * s2;
		qDot4 -= AHRSIMU->_beta * s3;
	}

	/* 7. Интегрирование производных кватерниона за период dt */
	AHRSIMU->_q0 += qDot1 * AHRSIMU->_sampleRate;
	AHRSIMU->_q1 += qDot2 * AHRSIMU->_sampleRate;
	AHRSIMU->_q2 += qDot3 * AHRSIMU->_sampleRate;
	AHRSIMU->_q3 += qDot4 * AHRSIMU->_sampleRate;

	/* 8. Нормализация кватерниона (удерживаем единичную длину) */
	recipNorm = oneOverSqrt(AHRSIMU->_q0 * AHRSIMU->_q0 + AHRSIMU->_q1 * AHRSIMU->_q1 + AHRSIMU->_q2 * AHRSIMU->_q2 + AHRSIMU->_q3 * AHRSIMU->_q3);
	AHRSIMU->_q0 *= recipNorm; AHRSIMU->_q1 *= recipNorm; AHRSIMU->_q2 *= recipNorm; AHRSIMU->_q3 *= recipNorm;
	
	calculateAngles(AHRSIMU);
}

/**
 * @brief  			Пересчёт углов Эйлера из кватерниона
 * @param  AHRSIMU  Указатель на структуру AHRSIMU_t
 */
static void calculateAngles(AHRSIMU_t* AHRSIMU) 
{
	/* 1. Вычисление углов в радианах из кватерниона (классические формулы Madgwick) */
	AHRSIMU->Roll = (float) fast_atan2f(AHRSIMU->_q0 * AHRSIMU->_q1 + AHRSIMU->_q2 * AHRSIMU->_q3, 0.5f - AHRSIMU->_q1 * AHRSIMU->_q1 - AHRSIMU->_q2 * AHRSIMU->_q2);
	AHRSIMU->Pitch = (float) fast_asinf(-2.0f * (AHRSIMU->_q1 * AHRSIMU->_q3 - AHRSIMU->_q0 * AHRSIMU->_q2));
	AHRSIMU->Yaw = (float) fast_atan2f(AHRSIMU->_q1 * AHRSIMU->_q2 + AHRSIMU->_q0 * AHRSIMU->_q3, 0.5f - AHRSIMU->_q2 * AHRSIMU->_q2 - AHRSIMU->_q3 * AHRSIMU->_q3);

	/* 2. Перевод в градусы и учёт магнитного склонения (смещение курса) */
	AHRSIMU->Roll *= AHRSIMU_RAD2DEG(1);
	AHRSIMU->Pitch *= AHRSIMU_RAD2DEG(1);
	AHRSIMU->Yaw = AHRSIMU->Yaw * AHRSIMU_RAD2DEG(1) - AHRSIMU->Inclination;
	
	/* 3. Ограничение Yaw в диапазоне [-180; 180] */
	if (AHRSIMU->Yaw < -180) 
	{
		AHRSIMU->Yaw = 180.0f - (-180.0f - AHRSIMU->Yaw);
	} 
	else if (AHRSIMU->Yaw > 180) 
	{
		AHRSIMU->Yaw = -180.0f - (180.0f - AHRSIMU->Yaw);
	}
}

/**
 * @brief  	  Быстрое вычисление обратного квадратного корня 
 * @param  x  Входное значение (> 0)
 * @return 	  1/sqrt(x)
 */
static float oneOverSqrt(float x) 
{
	return fast_inv_sqrtf(x);
}
