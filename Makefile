joystick_mouse.c: mouse
	gcc joystick_mouse.c -o joystick_mouse
mouse:
	echo "joystick_mouse.c compiled. Run with ./joystick_mouse"
clean:
	rm -f joystick_mouse
