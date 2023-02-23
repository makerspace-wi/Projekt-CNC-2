# CNC2 Holzfräse by Michael M.

<img width="400" alt="Box_2" src="https://user-images.githubusercontent.com/42463588/126619607-593ac9c1-d0a9-4283-843b-035fd4263459.jpg"> <img width="400" alt="Box_1" src="https://user-images.githubusercontent.com/42463588/126619556-e53ee3de-4409-4855-93c3-0f16d925f3c9.jpg">

# Vorworte
Es handelt sich dabei um eine <b>CNC-Sperrholz-Fräse</b> aus dem Make(CT Hacks) Magazin 1/2014 
und der Weiterentwicklung nach CNC14 (cnc14.de), mit einer Arbeitsfläche von 600mm x 1200mm, 
davon einer Fräsefläche von 550mm x 900mm und einer Verfahrhöhe von 70mm
### Einweisungen
Die CNC2-Fräse kann nur nach Einweisung und Freischaltung benutzt werden. Die Bedienung ist am Anfang sehr komplex, daher sollte unbedingt ein Projekt vorhanden sein und die Einweisung erfolgt daher auch in zwei Teilen.
- Der erste Teil besteht aus Vorführung der Fräse mit Erklärung über das vorhandene Projekt.
- Beim zweiten Teile fräst der Benutzer sein eigenes Projekt und wird dabei von dem Einweiser unterstützt.

### [ToolsLocker](https://github.com/makerspace-wi/ToolsLocker/wiki)
Im ToolsLocker Fach "CNC-Zubehör" (Freischaltung erforderlich) befindet sich ein Zubehörbehälter für die CNC2, bestehend aus: Computer Maus, Tastatur, Fernsteuerung, Werkzeughöhensensor, 2 Schraubenschlüssel 15er & 17er (Werkzeugwechsel) und einem 44teiliges Sorotec Stufenspannpratzenset M6 für 6er Nut ([Spann Pratzen Set Teileliste](doc/Pratzen_Set.pdf)), Pinsel,  4 x Überwurfmutter, Spannzangenset (1 x 3mm, 1 x 3,175mm, 2 x 4mm, 1 x 5mm, 2 x 6mm, 1 x 8mm) und Anleitung für den Spänesauger<p>
Bitte immer sicherstellen, dass nach der Arbeit alles - komplett - zurückgestellt wird.
  
<img src="https://user-images.githubusercontent.com/42463588/129173707-4acb66d5-490d-45fb-bf54-a5bc99593962.jpg" width="300" border = "0" alt="ToolsLockerbox">

### Thema Fräser ###
Jeder Nutzer der Fräse sollte sich einen eignen Satz (Holz) Fräser anschaffen, vielleicht auch mit jemandem teilen.<br>
Wir empfehlen einen Satz zweischneidiger Flach-Schaftfräser mit den Durchmessern 3mm, 4mm, 6mm und 8mm - ggf. in 2 Schnittlängen. Vielleicht auch einen Gravierfräser<br>
Bezugsquelle(n): https://www.sorotec.de/shop/Zerspanungswerkzeuge/sorotec-werkzeuge/2-schneider/Schaftfraeser-HOLZ/
  
Fräserauswahl: https://www.sorotec.de/webshop/Datenblaetter/fraeser/fraeser_verwendung_schaftfraeser.png  
  
### __HILFE! - Ich habe eine Achse in den Endstop gefahren__ :scream:<br>
Wenn dies passiert ist - bitte einfach die Ruhe bewahren! Die Software hat sofort alle Bewegungen gesperrt und am unteren Rand des Bildschirms wird auch darauf hingewiesen (in roter Schrift & blinkend). In diesem Fall die 'F11' Taste gedrückt halten und die betroffene Achse manuell aus dem Endstopp fahren - danach ist die Welt wieder in Ordnung.
  

----
  
## Typischer Fräsprojekt Ablauf ##

* [Zubehör Box aus ToolsLocker holen](#ToolsLocker)
* Fräse einschalten und mit RFID-Chip anmelden
   * [Einschalten](#Einschalten)
   * [Einloggen](#Freigabe-mit-RFID-Chip)
* Programm CNC Controller V11 starten
   * [Programm starten](#Estlcams-CNC-Controller-V11-starten)
* Fräser montieren  
  * Fräsmotor mittels Fernsteuerung, Maus oder Direkteingabe an eine geeignete Position fahren
  * passende Spannzange und Überwurfmutter aus der Zubehörbox entnehmen und in den Spindelmotor einsetzen (leicht andrehen) [Spannzangen](#Spannzangen)
  * Fräser tief genug einsetzen und dann mit den beiden Schraubenschlüssel die Überwurfmutter spannen
* Werkstück auf Arbeitsfläche einspannen 
  * [horizontales Spannen](#Horizontales-einspannen)
  * [vertikales Spannen](#Vertikales-einspannen)
* Spindelgeschwindigkeit einstellen
  * Die Geschwindigkeit mittels dem Rändelrad direkt am Motor einstellen. Die Positionen 1 bis 6 entsprechen 5.000 bis 25.000 U/min. D.h. die maximale Geschwindigkeit ist 25.000 Umdrehungen pro Minute und passt im allgemeinen für alle Holzarbeiten. Zur Berechnung der Vorschubgeschwindigkeit muss dann auch dieser Wert genommen werden.
  * [Spindelgeschwindigkeit](#Spindelgeschwindigkeit)
* Zu verarbeitende File mit Estlcam CNC-Steuerung öffnen
  * [USB-Stick mit File einstecken](#Datenträger-einstecken)
  * zu bearbeitende File auswählen (Estlcam Bildschirm rechts/oben)
  * [Bildschirmdarstellung](doc/Estlcam_Codeanzeige.jpg)
* Nullpunkt anfahren und Achsen 'nullen'
    * Fräser manuell direkt über dem Werkstücknullpunkt positionieren - der Fräser muss nicht das Werkstück berühren
    * Alle Achsen NULLEN - z.B. mit mittlerer Maus-Taste oder in der Estlcam Kommandozeile mit Befehl 'Null' 
* Z-Nullpunkt automatisch mit Sensor ermitteln

  * Sensortest

    ###  !!! vor dem Benutzen des Sensors testen, ob er funktioniert !!!: ###
    Testplatte an den Fräser (Gewind, Überwurfmutter) halten, wenn der Sensor auslöst (... F11 blinkt) mit F11 bestätigen, alles ok.
    
    ### wenn der Sensor nicht auslöst (... F11 nicht sichtbar) ist es nur manuell möglich den Nullpunkt einzustellen!!! ###

  <img src="https://user-images.githubusercontent.com/42463588/128348039-859f8c6a-bd0f-4af7-ab58-33d2fa53a9ae.jpg" width="300" border = "0" alt="Sensor">

* Absauge einschalten
* Fräsvorgang starten

  
## Fräse runter fahren (ausschalten)
<ol>
  <li>Referenzfahrt ausführen - es wird der Maschinennullpunkt ermittelt [Grundposition der Fräse]</li>
  <li>Programm Estlcam CNC-Steuerung schließen</li>  
  <li>PC herunterfahren</li>
  <li>Computer-Maus ausschalten</li>
  <li>Am Zugangsgerät abmelden</li> 
  <li>Steckdosenleiste ausschalten</li>
  <li>CNC zudecken mit grünem Tuch</li>
  <li>Zubehör zurück in den ToolsLocker</li>
</ol>

* [Referenzfahrt ausführen](doc/Estlcam_Referenz.jpg)
  
----
  
# Einschalten
__Tastatur, Fernbedienung und Computer-Maus (eingeschaltet) auf Brett bereit legen.__

Hinter der Fräse befindet sich eine Steckdosenleiste mit Schalter - mit diesem Schalter einschalten:

<img width="400" alt="Box_3" src="https://user-images.githubusercontent.com/42463588/127311146-514fe918-c521-48f9-8353-5dd8aeab8ffb.jpg">
<br><br>
Der PC bootet mit dem Einschalten automatisch.
Sollte das nicht funktionieren, bitte Taster (Start) am PC drücken.
  
<img width="400" alt="Box_2"  src=https://user-images.githubusercontent.com/42470750/203059431-db8376b1-765b-42ac-a8a4-8cc3a065d66d.jpg>
<br><br>
  
# Freigabe mit RFID Chip
Die Nutzung der CNC-Fräse ist nur nach Einweisung und Freischaltung des Mitglieds möglich.
Dazu bitte den roten RFID-Chip vor dem Lesegerät bewegen, bis die Aktivierung bestätigt wird. Bitte auf das Display achten, es könnte hilfreiche Hinweise liefern.

<img src="https://user-images.githubusercontent.com/42463588/128074604-47986882-cf10-4fdd-9c07-c87742d69098.jpg" width="300" border = "0" alt="Box_8">
  
# Estlcams CNC Controller V11 starten #
Zwischenzeitlich sollte der Mini-PC hochgefahren sein und den 'Desktop' zeigen.<br>
  
<img src="https://user-images.githubusercontent.com/42463588/128076185-05e76ea6-103b-4e6b-ade6-b90337c380f4.JPG" width="200" border = "0" alt="Box_9">

Dort dann das Programm &nbsp;<i>CNC Controller V11</i> &nbsp; mit Doppelklick starten - du erhältst folgenden Startbildschirm:
<br><br>
<img src="https://user-images.githubusercontent.com/42463588/128328089-174bd2f8-ad21-4cdb-ae79-928e339b2ae2.jpg" width="400" border = "0" alt="Box_11">

Die X-, Y- und Z-Position des Fräsmotors kann per Fernsteuerung, als auch per Mausklick auf die blauen Richtungspfeile gesteuert werden.<br>
Jedes Feld und jede Funktion wird ausführlich erklärt, wenn man den Mauszeiger darüber positioniert.

<img src="https://user-images.githubusercontent.com/42470750/203062942-5aad268b-3a6c-4efd-95e9-3add23fc373e.jpg" width="400" border = "0" alt="Box_16">

Sollten die roten Kreise am unteren rechten Rand des Fensters erscheinen, wurde das einloggen vergessen. (Die Steuerung ist aus!!!)


Nach laden der CNC-File: [G-Code & Werkstückdarstellung](doc/Estlcam_Codeanzeige.jpg)

# Datenträger einstecken {Laufwerk E}#
<img src="https://user-images.githubusercontent.com/42463588/128076473-a8692cfa-2d47-4370-b53b-c83a559315cd.jpg" width="300" border = "0" alt="Box_10">

Auf dem USB-Stick befindet sich die zu verarbeitende CNC-File. Wurde das Objekt mit Fusion360 designed und dem integrierten CAM-Prozessor der G-Code berechnet, hat die File die Endung .nc (z.B. 'Werkstueck.nc')

# Fräsmotor mit Fernsteuerung bewegen und Fräser montieren #


<img src="https://user-images.githubusercontent.com/42470750/203064244-56e8c240-c14f-400e-87f5-74218547eb0c.jpg" width="400" border = "0" alt="Box_17">

__Achtung: Nicht vergessen die große ENTER-Taste (rechts/unten) der Fernsteuerung zu drücken! Sie baut dann die Verbindung mit dem Mini-Computer auf.__


<img src="https://user-images.githubusercontent.com/42470750/203064278-7623f243-93f4-43d8-97d8-dd209a1b41d5.jpg" width="400" border = "0" alt="Box_18">

__Achtung: Sollte die Fräse nicht mit der Tastatur bewegen lassen, bitte die NUM-Taste drücken (Pfeile werden aktivert).__


# Werkstück einspannen #
### Horizontales einspannen ###
[Spannpratzen Set Teileliste](doc/Pratzen_Set.pdf)

<img src="https://user-images.githubusercontent.com/42463588/129189024-b84dbb0b-0e3f-42d5-b601-a581d749caa9.jpg" width="300" border = "0" alt="Praz2"> <img src="https://user-images.githubusercontent.com/42463588/128347999-0ca39f10-766b-45a8-beaa-dbfe2bed5d88.jpg" width="300" border = "0" alt="Box_12">

<img width="300" alt="Bildschirmfoto 2021-08-03 um 19 40 27" src="https://user-images.githubusercontent.com/42463588/128348315-10720b31-07db-4886-a029-c2f4f445cf3f.png"> <img width="300" alt="Bildschirmfoto 2021-08-03 um 19 41 06" src="https://user-images.githubusercontent.com/42463588/128348234-1c877c6b-0459-4ba3-af51-481c5ebb7b43.png"> <img width="300" alt="Bildschirmfoto 2021-08-03 um 19 40 59" src="https://user-images.githubusercontent.com/42463588/128348268-16e7ea15-e996-4bef-bb09-265867835e81.png">

### Vertikales einspannen ###
Unter der CNC-Fräse findet ihr zwei spezielle Spannzwingen, die vorher an der rechten Kante des Frästisches eingeführt und und fixiert werden müssen. Dieser Vorgang wird bei der Einweisung erklärt und gezeigt.

<img src="https://user-images.githubusercontent.com/42463588/127312994-6e0c8802-4759-4f5c-81bd-af389516bfb8.jpg" width="300" border = "0" alt="Box_5"> <img src="https://user-images.githubusercontent.com/42463588/127313679-64bd7a75-b21e-49f5-abad-8df5950e09c4.jpg" width="300" border = "0" alt="Box_6"> <img src="https://user-images.githubusercontent.com/42463588/127313493-08838aa2-08c9-4748-abaf-e10df9ba130d.jpg" width="300" border = "0" alt="Box_7" >


## Spannzangen ##
![IMG_8818](https://user-images.githubusercontent.com/42463588/129184681-d41899d7-4912-49a2-9a02-be3acaca4dcb.jpg)
Bestand am 12.08.2021:<br>Spannzangenset für folgende Schaftdurchmesser: 1 x 3mm, 1 x 3,175mm, 2 x 4mm, 1 x 5mm, 2 x 6mm, 1 x 8mm) und 4 Überwurfmuttern
  
## Spindelgeschwindigkeit ##

<img src="https://user-images.githubusercontent.com/42463588/129193501-23b1e0ee-7be5-4313-85db-652f2644a2e0.jpg" width="300" border = "0" alt="Spin_1" > <img src="https://user-images.githubusercontent.com/42463588/129193540-4cce49d8-bcd4-492c-9d8d-f17a5b66a61e.jpg" width="300" border = "0" alt="Spin_2" >
  
Die Spindelgeschwindigkeit wird direkt durch das Rändelrad am Fräsmotor eingestellt. Sie hängt ab vom zu bearbeiteten Material und dem genutzten Fräser.
  
![Fräserauswahl](/images/fraeser_verwendung_schaftfraeser.png)<br>
[Schnittwerteberechnungen](doc/schnittwerte.pdf)

  


## Weiterführende Dokumentationen

[Rampa Muffen Einbau](doc/RampaMuffen%20Einbau.pdf)

[Schaltbilder CNC-Steuerung](doc/CNC_SteuerungGRBL_SSR_V2.pdf)

[Zeichnung Riemenschutz](doc/Riemenschutz%20Zeichnung%20v2.pdf)

[Schnittwerteberechnungen](doc/schnittwerte.pdf)

[Spann Pratzen Set Teileliste](doc/Pratzen_Set.pdf)

----



