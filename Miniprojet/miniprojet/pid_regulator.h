#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//they are here in case we use them in main
enum STATE {
	SEARCH, TARGET, CHARGE, TURNAROUND, GOBACK
};

//start the PI regulator thread
void pid_regulator_start(void);

#endif /* PI_REGULATOR_H */
