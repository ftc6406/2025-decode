package mousemotion.src.eventclassification.eventcodes;

import java.util.HashMap;

import mousemotion.src.eventclassification.EventCategory;
import mousemotion.src.eventclassification.EventTypes;

public interface EventCode extends EventCategory{
    public HashMap<Integer, EventCode> VALUE_MAP = new HashMap<>();
    public EventTypes getEventType(); 

    public static EventCode byValue(int value) {
        return VALUE_MAP.get(value);
    }
}