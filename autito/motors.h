#include <Arduino.h>

#define DIR1PIN_DEFAULT 7
#define DIR2PIN_DEFAULT 6
#define ENABLEPIN_DEFAULT 5

class motor
{
	private:
		int dir1Pin_,dir2Pin_,enablePin_;//TODO const tal vez
		int speed_;
		bool forward_;
	public:
		//contructores
		motor();
		motor(int d1, int d2, int en);
		motor(const motor &);
		~motor();
		// TODO Operadores sobrecargados.
		bool operator==(const motor &)const;
		bool operator!=(const motor &)const;
		motor & operator=(motor const &);
		void operator()(int, bool);
		//getters y setters
		int getSpeed();
		void setSpeed(int);//TODO const puede ser necesario
		void setDirection(bool);

		void initialSetupForArduino();
		void setStateForArduino();
};

void motor::initialSetupForArduino()
{
	pinMode(dir1Pin_,OUTPUT);
	pinMode(dir2Pin_,OUTPUT);
	pinMode(enablePin_,OUTPUT);
}

void motor::setStateForArduino()
{
	analogWrite(enablePin_, speed_);
	if(forward_ == true)
	{
	    digitalWrite(dir1Pin_, LOW);
        digitalWrite(dir2Pin_, HIGH);
	}
	else
	{
	    digitalWrite(dir1Pin_, HIGH);
        digitalWrite(dir2Pin_, LOW);
	}
}

//Contructores
motor::motor(): dir1Pin_(DIR1PIN_DEFAULT), dir2Pin_(DIR2PIN_DEFAULT), enablePin_(ENABLEPIN_DEFAULT), speed_(0), forward_(true)
{
	initialSetupForArduino();
	setStateForArduino();
}
motor::motor(int d1, int d2, int en): dir1Pin_(d1), dir2Pin_(d2), enablePin_(en), speed_(0), forward_(true)
{
	initialSetupForArduino();
}
motor::motor(const motor & m)
{
	dir1Pin_  = m.dir1Pin_;
	dir2Pin_ = m.dir2Pin_; 
	enablePin_ = m.enablePin_;
	speed_ = m.speed_;
	forward_ = m.forward_;

	initialSetupForArduino();
	setStateForArduino();
}

motor::~motor()
{}

//Sobrecarga de operadores
motor & motor::operator=(motor const &m)
{
	if(this != &m)
	{	
		dir1Pin_  = m.dir1Pin_;
		dir2Pin_ = m.dir2Pin_; 
		enablePin_ = m.enablePin_;
		speed_ = m.speed_;
		forward_ = m.forward_;

		initialSetupForArduino();
		setStateForArduino();
	}
	return *this;
}

void motor::operator()(int s, bool f)
{
	speed_ = s;
	forward_ = f;
	setStateForArduino();
}

bool motor::operator==(const motor & m)const
{
	if( speed_ == m.speed_ && forward_ == m.forward_ )
		return true;
	else
		return false;
}

bool motor::operator!=(const motor & m)const
{
	return (*this == m)? false: true;
}


//getters y setters
int motor::getSpeed()
{
	//TODO algo con la lectura del encoder
	return speed_;
}
void motor::setSpeed(int s)
{
	speed_ = s;
	setStateForArduino();
}
void motor::setDirection(bool f)
{
	forward_ = f;
	setStateForArduino();
}


