Allgemeines:
Ich habe den alten Code(2016/17) als Grundlage verwendet.
 Ich habe ihn aber nicht vollständig verwendet bzw. Sachen anders gemacht. 
Das hat zur Folge, dass einige Funktionen, Variablen, Properties zwar noch im Code stehen, aber von mir eigentlich nicht verwendet
wurden. Der Filter fasst eigentlich zwei Verschiedene Filter zusammen, die in auch mal getrennt waren. Für die Übersichtlichkeit wäre es besser,
sie wieder zu trennen. Das ist zum einem Adative Cruse Control, also die Abstandshaltung und ObjectDetection, also in gewissen Situationen in gewissen Bereichen
nach Objecten zu suchen.
Der Filter ist Time-Triggert, es wäre allerdings besser, ihn auf z.B. Ultraschall zu triggern
Ich habe beide Funktionalitäten mit dem Lidar realisiert. Der Teil Ultrasonic im Namen ist historisch bedingt. Ultraschall wurde nicht verwendet.
Allgemeines zum Lidar:
Prinzipiell war ich mit den Ausgaben vom Lidar sehr zufreden. Allerdings sollte man zwei Dinge im Hinterkopf haben:
1) Wenn etwas direkt vor dem Auto steht (wirklich direkt davor) wird es nicht erkannt
2) Schwarze Objekte werden teilweise auch nicht erkannt. Das hat zur Folge, dass z.B. von einem Auto lediglich die Ultraschallsensoren gesehen werden.
3) Ich habe nie gesehen, dass ein Punkt erkannt wurde, wo absolut nichts war. Allerdings gab es Rauschen, ein Objekt vorhanden ist (siehe Datenblatt Lidar)
Zu meinen verwendeten Koordinatensysteme: 
zumindest bei uns war der Winkel der Lidarpunkte nicht sehr intuitiv, deshalb haben wir die Funktion CompansateAngle eingeführt.
rechts ist 90° und links -90°. Mir ist leider erst später aufgefallen, dass das nicht mathematisch definiert ist. Wenn ich mich später auf x, y beziehe,
beziehe ich mich auf das mathematische Koorninatensystem mit horizontal x (rechts pos und links neg) und vertikal y (vom Auto aus nach vorne pos und nach hinten neg)
Das ist NICHT das sonst übliche Autokoordinatensystem.
1)
InputPins:
-ultraonicInput: der Ulatrasonic, da dieser relativ unzuverlässig war (hat manchmal ohne Grund 0 ausgeben) wurde er davor in UltrasonicMean gemittel.
Letztlich wurde er von uns nicht verwendet. Die Funktionalität des Überholen (später näher beschrieben) kann auch an eine andere Stelle eingebaut werden
-actionInput: Actions der State Machine
-targetSpeedInput: Sollgeschwindigkeit aus Select Speed (früher gab es zwei, da wir SelectSpeed eingebaut haben, haben wir es nicht mehr gebraucht)
-steeringAngleInput: wurde auch nicht mehr verwendet, an sich hilfreich, wenn an die Hinderniserkennung in der Kurve mit dem Lenkwinkel macht;
auch aus Select Speed
-laserscannerIn: LaserscannerInput
-laneDetectionLine: Line aus der lane Detection, um in der Kurve Hindernisse in der Fahrspur zu erkennen
OutputPins:
-targetSpeedOutput: Ausgangssgeschwindigkeit an den PID Controller
-feedback Output: Feedback an die State machine
-laserOut: Laser Output für Debugzwecke; hat leider nicht funktioniert
-GuardrailPosition: Wir sind auf der Rampe anhand der Begrenzung gefahren, das war die Information, wo die Rechte Begrenzung sich befindet.
Es wurde an Move To Point geschickt
Video
-inputVideo: aus fisheyeundistortion
-outputVideo
-outputVideoImageROI: zeigt in dem Kamerabild die ROI an. hat allerdings nur mittelmäßig funktioniert

 
ACC:
Ziel: Die Geschwindigkeitsregulierung in Abhängigkeit von dem nächsten relevanten Objekt
Wird bei ProcessTargetSpeed() berechnet. Die Idee ist, das nächste relevante Objekt zu bestimmen. 
Aus einem Look-Up-Table wird anschließend mit einer Interpolation eine maximale Geschwindigkeit berechnet.
Die geringere der beiden Geschwindigkeiten wird übermittelt.
Also von Anfang an: zunächst wird die Sollgeschwingkeit ausgelesen. 
Dann wird unterschieden (anhand des Vorzeichens), ob das Auto vorwärts oder rückwärts fahren soll.
Fürs Vorwärtsfahren: die Lidardaten werden ausgewertet. In ObstacleDetectionWithLidar wollte ich den Lidarpunkten Objekten zuordnen. 
Das mit nicht so gut funktioniert.
Ich habe es aber auch nicht mehr gebraucht. Außerdem werden die Lidardaten dem Winkel nach sortiert. 
Als nächsten wird in CalcClosestRelevantObstacle() das nächste relevante Objekt bestimmt. Hiebei wird wunterschieden, ob man sich gerade in einer Kurve
befindet. Das habe ich mit Hilfe von Informationen von laneDetection gemacht. In den Jahren vorher wurde das mit dem Lenkwinkel gemacht (bzw. es wurde nicht 
unterschieden, sondern immer Berücksichtig)
Zunächst einmal: ich hatte zeitweise implementiert, dass mehrere Punkte, die benachtbart sein müssen, Voraussetzung für ein relevantes Objekt sind.
Allerdings sieht das Lidar wie gesagt bei einem Auto nur die US-Sensoren. Deshalb wollte ich auf Nummer sicher gehen. Und es gab auch nie den Fall,
dass auf ein Objekt detektiert wurde, wo nichts war. Also berücksichtige ich wirklich nur einen (den nächsten) Punkt.
Wenn das Auto geradeaus fährt, wird die Distanz folgendermaßen berechnet:
Es wird bei theoretisch möglichen Punkten (!= 0.0 und < Threshold(ich glaube bei mir 2m)) mit der Funktion CalcAngleFromDistance() der
maximale Winkel berechnet. Das ist der maximale Winkel, den das der Punkt be gegebener Entfernung haben darf, damit er sich noch vor dem Auto befindet.
Dafür wird der asin(Halbe Auto Breite/Distanz zum Punkt) in Grad berechnet. Anschließend wird bei dem Punkt geschaut ob der tasächliche Winkel betragsmäßig
kleiner ist, das dieser berechnete Winkel. Wenn ja, ist es ein relevanter Punkt. Dann wird verglichen, ob der es der bis dahin nächste Punkt ist. Das wird
mit allen Punkten gemacht. So wird der Globale nächste relevante Punkt bestimmt. Man hätte es auch mit der bei einer Kurve verwendeten Funktion
IsObjectInBox() machen können.
Bei einer Kurve (im Code: else //taking a bend) hatte ich zu Begin die Idee, dies mit Hilfe von Informationen von LaneDetection machen können.
Der Filter bekommt die INFOrmationen, wo die Fahrspur sich befindet. Es sind zwei Geraden. An diese Geraden werden in einem gewissen Abstand
Boxen reingelegt. In diesen Boxen werden der Reihe nach nach Lidarpunkten gesucht. Das Sollte mit IsObjectInBox() gemacht werden.
In dieser Funktion werden für in Frage kommende Lidarpunkte x (f32rcos) und y (f32rsin) berechnet und mit der übergebenen ROI (Region of interest) vergliechen.
Es wird der nächste Entfernung innerhalb dieser übergebenen ROI zurückgegeben. 
Leider hat es mit den Boxen nicht geklappt. Wir sind relativ langsam gefahren. Deshalb war es vertretbar, in einer Kurve lediglich 40cm vor dem Auto zu checken.
Die so berechnete Distanz wird in GetRedPerscentFront übergeben. Außerdem noch der aktuelle Modus mit übergeben.
Es gab 4 verschiedene Modi. DRIVING_MODE_ACTIVE ist der default Mode. Das Auto fährt gerade aus.
Bei MtP_MODE_ACTIVE wird die Geschwindigkeit faktisch ei9nfach weitergegeben. Bei dem DRIVING_MODE_CURVE
fährt das Auto gerade in der Kurve (man könnte es eigentlich auch weglassen).
Wir hatten eine Rampe, die wir rauffahren mussten, wenn diese erkannt wurde, wurde dieser Modus eingeschalten. (nicht besonders getestet)
Zurück zum ACC: Von GetRedPerscentFront wird eine interpolierte Geschwindigkeit ausgegebenn.
Je nach Modus wird ein anderer Look-Up-Table (z.B. bei DRIVING_MODE_ACTIVE m_vecXValuesDriving, m_vecYValuesDriving) verwendet, um in Abhängigkeit der
Distanz die Geschwindigkeit zu berechnen.
Ich habe die Look-Up-Table aus Vektoren ausgelesen. Es wäre besser, diese Vektoren aus einer XML auszulesen.
Mit ChooseOutputSpeedFront wird der kleinere der beiden Geschwindigkeiten (Eingang von anderem Filter und Interpolierte Geschwindigkeit) ausgegeben.
Moving backwards habe ich aus den Jahren davor übernommen. Es wurde aber nicht verwendet, da wir nur mit Move to Point rückwärts gefahren sind.
Überholen:
In ProcessUS() wird geschaut, ob ein Feedback zum Überholen ausgeben werden muss. Es wird geschaut, ob das Auto mit der Eingangsgeschwindigkeit
(GetLastInputTargetSpeed()) fahren müsst, aber der Ausgang (GetLastModifiedTargetSpeed()) zu klein zum Fahren ist.
Wenn dass der Fall ist, wird der ObstacleNoMovementCounter erhöht. Wenn dieser eine Gewisse Grenze überschreitet, wird ein Feedback zum überholen gesendet.
Wenn das Auto zwischenzeitlich wieder fährt, wird der ObstacleNoMovementCounter wieder zu 0 gesetzt.
Im Finale kam bei uns ein Verkehrsschild direkt nach dem Überholen. Das haben wir nicht geschafft. Dafür muss man entweder die Karte verwenden, oder
den Überholvorgang optimieren (nicht den gerade beschriebe Vorgang mit dem ObstacleNoMovementCounter)

Ablauf:
Während wir mit Move to Point gefahren sind, war der Filter faktisch ausgeschalten. Das war eigentlich nicht so gut. 
Es ist eine Überlegung Wert, den Filter wenn es geht vielleicht nur beim Parken auszuschalten (MtPMode).

ObstacleDetection:
Die Aufgabe ist, in manchen Situationen gewisse Bereiche (ROI) zu überprüfen, ob sich darin ein Objekt befindet.
Beispielsweise wird, wenn man an einer Vorfahrtsstraße links abbiegen möchte, der Gegenverkehr gegenüber überprüft.
Sobald das Schild erkannt wird, wird dem Filter der Befehl Check_Oncoming_Traffic geschickt. Das wird mit SetLastActionInput gespeichert.
Wenn das Auto sich ungefähr bei dem Schild befindet, bekommt der Filter den Befehl CHECK_ROI_INTERSECTION. 
Hier wird dann in diesem Fall ProcessActionIntersectionOncomingTraffic() ausgeführt. Das führt dann die Funktion ProcessROI mit der in den Parametern
definierten Werten für Oncoming Traffic aus. ProcessROI() ist eine Überladene Funktion, die 1, 2, oder 3 ROIs übergeben bekommt.
Mit dem zwei Befehlen, ist es relativ umständlich, wenn ihr es anderes machen wollt, könnt ihr das gerne tun.
ProgecessROI() funktioniert folgendermaßen: das enum occupancy gibt den Status der ROI an. Zunächst wird der Status auf INITIALIZED
gesetzt. Anschließend wird/werden die ROI(s) auf Objekte hin untersucht. Das geschieht in CheckROIforObstacles().
Hier wird wiederrum IsObjectInROI. Das bekommt zum einen die Lidardaten (Obstacles) übergeben, zum anderen die zu untersuchende ROI. Es gibt 0 zurück, wenn
sich kein Objekt in der ROI befindet und etwas != 0, wenn sich ein Lidarpunkt darin befindet.
Hier wird der IntersectionMode verwendet. Wenn das Auto beispielsweise direkt aus einer Linkskurve kam, hat es an einer anderen Stelle (ein bisschen vor dem Schild)
die ROIs überprüft. Wenn es direkt aus einer geraden Strecke kam, wurden die ROIs direkt am Schild überprüft. Aus diesem Grund bekam der Filter über
eine Action mitgeteilt, in welcher Situation es sich befindet. Das wurde dann bei der y-Koordinate der ROI berücksichtigt.
Anschließend wird wie oben bei jedem Lidarpunkt überprüft, ob die x-Koordinate (f32rcos) und die y-Koordinate (f32rsin) sich in der ROI befinden.
Ich habe als Threshold wieder 1 genommen. Da wie oben beschrieben, der Lidar relativ robust war und es teilweise sonst nicht immer funktioniert hat (schwarz wird nicht erkannt...)
Wenn eine der bis zu 3 möglichen IsObjectInROI eine Zahl zurückgibt, wird der Status auf OCCUPIED_SPACE gesetzt. Wenn es m_propUI32FreeROIBoundery oft
darauf gesetzt wurde, wird der Status auf OCCUPIED_STATIC gesetzt. Wenn alle ROIs frei sind, ist es ähnlich. Hier ist der Status zunächst STATE_PENDING und dann FREE_SPACE.
leider ist mir erst recht spät aufgefallen, dass nur ein LaserInput verwendet wird, also genügt eigentlich ein durchlauf. Es ist eine Überlegung wert, ob man nicht immer wartet, bis
neue Laserscannerdaten kommen. Anderseits hat es bei mir immer geklappt, es ist nur nicht der Sinn der Sache.
Wenn als Status OCCUPIED_STATIC zurückgegeben wird wird ein Feedback gesendet (Static Obstacle), wenn 
FREE_SPACE zurückgegeben wird, wird NO_OBSTACLE als feedback gesendet.
Bei ROIs von nicht Kreuzungen funktioniert es im Prinzip genau so.

Wie gesagt, mit den ganzen Unterfunktionen, ist es unnötig kompliziert :( Tut mir leid. Man kann das sicher noch vereinfachen.

Zur Rampe: die rampenerkennung hat nicht funktioniert. Ich hatte im Finale keine Zeit sie zu testen.

Viel Spaß!