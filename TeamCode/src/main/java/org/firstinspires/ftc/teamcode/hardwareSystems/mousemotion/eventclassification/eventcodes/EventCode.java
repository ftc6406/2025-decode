package org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.eventclassification.eventcodes;

import java.util.HashMap;

import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.eventclassification.EventCategory;
import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.eventclassification.EventTypes;

public interface EventCode extends EventCategory{
    public HashMap<Integer, EventCode> VALUE_MAP = new HashMap<>();
    public EventTypes getEventType(); 

    public static EventCode byValue(int value) {
        return VALUE_MAP.get(value);
    }
}
