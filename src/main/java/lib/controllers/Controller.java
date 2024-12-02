package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class Controller {
    protected final Joystick controller;

    public Controller(int port) {
        this.controller = new Joystick(port);
    }

    public Trigger get(BooleanSupplier sup) {
        return new Trigger(sup);
    }
}
