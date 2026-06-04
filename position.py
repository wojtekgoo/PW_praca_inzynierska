"""
position.py — współdzielona klasa Position, uzywana przez wszystkie lokalizatory i Filtr Kalmana

Wszystkie lokalizatory zwracają obiekt klasy Posittion, który jest następnie podany na Filtr Kalmana

Pola:
    lat       : latitude (szerokość geograficzna) w stopniach
    lon       : longitude (długość geograficzna) w stopniach
    accuracy  : szacowana dokładność horyzontalna pomiaru w metrach (1-sigma)
                Im mniejsza, tym bardziej brana pod uwagę przez Filtr.
    source    : żródło, które dostarczyło fixu, np. "uwb", "gnss", "wifi",
    timestamp : unix timestamp kiedy nastąpił fix

Accuracy jest najważniejszym polem dla Filtru Kalmana. Jest ono użyte wprost jako odchylenie standardowe danego pomiaru.
Typowe wartości:
    UWB             0.10 – 0.50 m
    GNSS (good)     3    – 10   m
    GNSS (degraded) 10   – 30   m
    WiFi            15   – 100  m
    Cell history    500  – 1500 m
    Cell single     1500 m
"""

import time
from dataclasses import dataclass, field


@dataclass
class Position:
    lat:       float          
    lon:       float          
    accuracy:  float          
    source:    str            
    timestamp: float = field(default_factory=time.time)

    def age(self) -> float:
        """ilość sekund odkąd mamy fix"""
        return time.time() - self.timestamp

    def __str__(self) -> str:
        return (
            #f"[{self.source.upper():16s}] "
            f"[{self.source.upper()}] "
            f"lat={self.lat:.6f}  lon={self.lon:.6f}  "
            f"accuracy=~{self.accuracy:.1f}m  "
            f"age={self.age():.1f}s"
        )
