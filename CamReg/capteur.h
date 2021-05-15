#ifndef CAPTEUR_H_
#define CAPTEUR_H_

//start le thread Capteur
void capteur_start(void);
//renvoie la valeur de la distance a l'objet
uint16_t get_object_distance(void);

#endif /* CAPTEUR_H_ */
