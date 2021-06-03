
#ifndef LPID__HPP
#define LPID__HPP

class LPID
{
public:
	LPID() {
		this->Kp = 0;
		this->Ki = 0;
		this->Kd = 0;
		this->setPoint = 0;
		this->all_error = 0;
		this->last_error = 0;
		this->last_last_error = 0;
		this->sameple_time_ms = -1;
	}
	LPID(double Kp, double Ki, double Kd, double time_ms = -1, double setPoint = 0) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		this->setPoint = setPoint;
		this->all_error = 0;
		this->last_error = 0;
		this->last_last_error = 0;
		this->sameple_time_ms = time_ms;
	}

// ��ࣺĿ��-����
// difference = target - value
	double PIDControl_byDiff(double difference, int way = Incremental) { // ���ݲ�ֵ�����
		double control;
	// P���ڣ�����
		double Pitem = this->Kp * difference;

		// I���ڣ�����
		double Iitem = 0;
		switch (way)
		{
			case Position: // λ��ʽ�������
				Iitem = this->Ki * this->all_error;
				break;
			case Incremental: // ����ʽ���˴����ϴ������ϴ����
				Iitem = this->Ki * (difference + this->last_error + this->last_last_error);
				break;
			default:
				break;
		}

		// D���ڣ�΢��
		double Ditem = this->Kd * (difference - this->last_error);
		control = Pitem + Iitem + Ditem;

		// ����ۻ�
		this->all_error += difference;
		this->last_last_error = this->last_error;
		this->last_error = difference;

		return control;
	}

	double PIDControl_byValu(double value, int way = Incremental) {
		return this->PIDControl_byDiff(this->setPoint - value, way);
	}
	double sameple_time_ms; // ÿ�ε��ڵ�ʱ��,��λ ms

	enum ways
	{
		Position,
		Incremental
	};

	
	void set_p(double p) {
		this->Kp = p;
	}
	void set_i(double i) {
		this->Ki = i;
	}
	void set_d(double d) {
		this->Kd = d;
	}
private:
	double Kp, Ki, Kd, setPoint;

	// λ��ʽPID�� ������״̬�й�
	double all_error;

	// ����ʽPID�� ��˴κ�ǰ����״̬�й�
	double last_error, last_last_error;
};

#endif // !1


