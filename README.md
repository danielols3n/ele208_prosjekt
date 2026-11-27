# Semesterprosjekt ELE208 – Oppgave 1: Vindusvaskerobot

Semesterprosjekt i **ELE208 Robotmanipulatorer** ved Høgskulen på Vestlandet (HVL), høsten 2026.

Gruppen har valgt oppgave 1: et robotsystem som vasker vinduer på en fasade. Løsningen er en **bomlift med en robotarm** på tuppen av bommen. Et 3D-kamera finner skitne flekker på glasset, og armen fører en vanndyse over flekkene.

![Simulering av roboten](Bilder%20+%20videoer/motion_planning1.png)

## Robotsystemet

Hele systemet er modellert som én seriekjede med 9 ledd (`full = lift * arm`) i Peter Corkes Robotics Toolbox:

| Del | Ledd | Beskrivelse |
|-----|------|-------------|
| Lift | 1 | Rotasjon av bommen (yaw) |
| Lift | 2 | Vinkling av bommen (pitch), maks 15° under horisontal |
| Lift | 3 | Teleskopbom (prismatisk), 3,6–15 m |
| Lift | 4 | Plate ved bomtuppen, holdes loddrett mekanisk (`q4 = q2 − π/2`) |
| Arm | 5–9 | UR5-inspirert arm med 5 ledd (UR5 sitt ledd 6 er fjernet), med dyse som verktøy |

- **Liftbase:** `[-3 -2 1.5]` m i verdenskoordinater, utenfor veggen.
- **Kamera:** montert på platen ved bomtuppen, 0,3 m under origo i frame 4.
- **Arbeidsområde:** 10 × 10 m vindusfasade.
- **Krav som sjekkes:** dysefart på minst 10 cm/s under vasking, og at vannstrålen når hele arbeidsområdet (dysevinkel maks 45°).
- Under selve vaskingen står liften i ro, og bare armen beveger seg.

## Filer

| Fil | Innhold |
|-----|---------|
| [`kinematics_model.m`](kinematics_model.m) | Hovedscriptet: DH-modell av lift og arm, dynamiske parametere, invers kinematikk (`ik_system`), transformasjon fra kamera til robot, leddynamikk og overføringsfunksjon for skulderleddet, hastighetsregulator (P og PI), posisjonsregulator (P), arbeidsområde med vegg, baneplanlegging, animasjon, RNE på en vaskebane, dysefart, kapasitetssjekk og simulering av rekkevidden til vannstrålen. |
| [`window_washer_planner.m`](window_washer_planner.m) | Baneplanlegger med kollisjonssjekk. Lager flekker på vinduene, planlegger løft- og armbevegelser og vaskestrøk, og returnerer en trajektorie (`res.traj`) som kan sendes rett til `fkine`, `rne` eller ROS. |
| [`gazebo_vaskebane.m`](gazebo_vaskebane.m) | Kjører vaskebanen fra planleggeren på roboten i Gazebo over ROS 2 (`FollowJointTrajectory`), logger målt posisjon og sammenligner den med den planlagte. |
| [`utregning_dynamic_model.py`](utregning_dynamic_model.py) | Overslag over masse og tyngdepunkt for bomseksjonene (aluminiumsprofiler), som brukes som dynamiske parametere i MATLAB-modellen. |
| [`Gazebo_model/`](Gazebo_model/) | Den endelige ROS 2-pakken `lift_window_sim`, som er brukt til testene i rapporten (kap. 4.7). Inneholder robotmodellen (URDF/xacro, bygget fra DH-tabellen og massene i MATLAB-modellen), fasadeverdenen med vannstrålens kraft (`facade_load.sdf`), kontrolleroppsett og `start_gazebo.sh`, som bygger pakken og starter Gazebo. Mappen `src/lift_window_sim/matlab/` har testscriptene `gaa_til_start.m` (kjører roboten til startposen for en test), `kjor_test.m` (sender en testbane til Gazebo og logger ønsket og målt leddposisjon) og `vurder_logg.m` (regner ut følgefeil, treffpunkt på veggen og dekning fra loggen). |
| [`Gazebo_model/test_results/`](Gazebo_model/test_results/) | Loggene og analysene (`*_analyse.json`) fra de tre Gazebo-testene i rapporten: rasterbane (`top`), dysevinkler (`vinkel`) og lastetest (`last`). |
| [`Gazebo_model_V10/`](Gazebo_model_V10/) | Forrige versjon av Gazebo-modellen (seks teleskoptrinn, glasset i `y = 1.97`). `gazebo_vaskebane.m` er skrevet for denne versjonen. |
| [`Presentations/`](Presentations/) | Presentasjonene fra PDR, CDR og sluttpresentasjonen. |
| [`Bilder + videoer/`](Bilder%20+%20videoer/) | Figurer fra baneplanleggingen, reguleringen, RNE-analysen og Gazebo-kjøringen. |

## Krav

- MATLAB med **Control System Toolbox** og **ROS Toolbox** (ROS Toolbox trengs bare for Gazebo-delen)
- **Robotics Toolbox for MATLAB** av Peter Corke
- **ROS 2 Jazzy og Gazebo** med xacro, for pakken i `Gazebo_model/`. Gazebo-pakken inneholder bare roboten; all styring skjer fra MATLAB.
- Python 3 (bare for `utregning_dynamic_model.py`)

## Slik kjører du

### Simulering i MATLAB

```matlab
kinematics_model                          % lager robotmodellen `full` og kjører alle analysene
res = window_washer_planner(full);        % lager vaskebanen
```

Planleggeren kan styres med valgfrie innstillinger:

```matlab
res = window_washer_planner(full, struct('nSpots', 10, 'showAnimation', true, 'exportCSV', false));
```

| Innstilling | Standard | Betydning |
|-------------|----------|-----------|
| `nSpots` | 20 | Antall skitne flekker som skal vaskes |
| `randomSeed` | 7 | Frø for tilfeldig plassering av flekkene |
| `showAnimation` | – | Vis animasjon av vaskingen |
| `exportCSV` | `true` | Lagre trajektorien som CSV |

### Gazebo / ROS 2

1. Bygg pakken og start Gazebo (krever ROS 2 Jazzy og `ros-jazzy-xacro`):
   ```bash
   cd Gazebo_model
   bash start_gazebo.sh
   ```
   Vent til terminalen skriver `TOP READY` (eller `CHAIN NOT READY`, se under). Bruk samme `ROS_DOMAIN_ID` i MATLAB og i Gazebo (24 er standard i ELE208).
2. Kjør testene fra MATLAB, med `Gazebo_model/src/lift_window_sim/matlab/` som arbeidsmappe:
   ```matlab
   gaa_til_start("top")          % kjør roboten til startposen (trengs hvis Gazebo skrev CHAIN NOT READY)
   logg = kjor_test("top");      % rasterbane over øverste 10 × 1 m-stripe
   r = vurder_logg(logg, "top"); % følgefeil, treffpunkt på veggen og dekning
   ```
   De andre testene er `"vinkel"` (dysevinkler på ±45°) og `"last"` (holder stillingen i 60 s med dyse, slange og vannstrålekraft). Start Gazebo på nytt mellom testene. `vurder_logg` lagrer resultatet som en JSON-fil ved siden av loggen.

Gazebo-modellen bruker samme leddvinkler og koordinater som MATLAB-modellen (glasset ligger i `y = 0`), bortsett fra at `boom_extension` er uttrekket per teleskoptrinn: `d3 = 3.6 + 4·boom_extension`. Alle fire teleskoptrinn posisjonsstyres med samme uttrekk. For at Gazebo skal håndtere teleskopet, er 20 kg flyttet fra det ytterste trinnet til hvert av de tre indre; totalmassen og gravitasjonsmomentet i startposen er de samme som i MATLAB.

### Resultater fra Gazebo-testene

| Test | Resultat |
|------|----------|
| Rasterbane (øverste 10 × 1 m) | Dysens følgefeil maks 11,4 mm (RMS 10,5 mm), treffpunkt på veggen maks 11,3 mm fra planlagt bane, hele stripen dekket med antatt strålebredde 13 cm |
| Dysevinkler (R3) | Alle fire ±45°-mål nådd innenfor 0,06° |
| Lastetest (R4–R6) | Største drift siste 10 s: 0,02° på arm- og liftledd, 0,005 mm på teleskopet |

### Hele vaskebanen (forrige Gazebo-versjon)

`gazebo_vaskebane.m` kjører hele vaskebanen fra planleggeren i Gazebo. Den er skrevet for den forrige modellen i `Gazebo_model_V10/` (kontrolleren `joint_trajectory_controller`, glasset i `y = 1.97`), og må startes med den pakken:
```matlab
kinematics_model
res = window_washer_planner(full);
gazebo_vaskebane
```
Hvis `full` eller `res` mangler, lager `gazebo_vaskebane.m` dem selv.

## Bruk av KI

Hver fil starter med en deklarasjon av hvordan KI er brukt. Kort sagt er mesteparten av kinematikk- og dynamikkoden skrevet av gruppen selv. KI er brukt til plott, feilsøking og som sparringspartner, og mer omfattende i baneplanleggeren, Gazebo-scriptet og simuleringen av vannstrålen. Seksjoner skrevet med KI-støtte er merket i koden.

Denne README-filen er skrevet av Claude Code. I tillegg er prosjektet i dette github repositoryet satt opp med Claude Code og noen commits er utført med hjelp av Claude Code
