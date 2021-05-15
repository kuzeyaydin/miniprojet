#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

enum STATE {
	SEARCH, CHARGE, TURNAROUND, GOBACK
};

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
