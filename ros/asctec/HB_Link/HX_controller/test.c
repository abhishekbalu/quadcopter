#include <stdlib.h>
#include <SDL/SDL.h>
 
int main(int argc, char **argv)
{static SDL_Event evenements;
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
        return EXIT_FAILURE;

    SDL_JoystickEventState(SDL_ENABLE);
    SDL_Joystick *joystick; // on crée le joystick
    joystick = SDL_JoystickOpen(0); // on l'assigne au numéro 0
    int i=0;
	while(i<2000){
	SDL_WaitEvent(&evenements);
	 switch(evenements.type){
    		case SDL_JOYBUTTONDOWN:
        	printf("Bouton %d\n",evenements.jbutton.button);
		i++;
		break;
	
		case SDL_JOYAXISMOTION:
			printf("Stick %d, valeur %3d \n", evenements.jaxis.axis,evenements.jaxis.value);
		i++;
		
		break;
	

		}
	}
	
    SDL_JoystickClose(joystick);
    SDL_Quit();
    return EXIT_SUCCESS;
}
