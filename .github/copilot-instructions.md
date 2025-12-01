<!-- Copilot/AI agent instructions for AZB_Raumschiff -->
# Kurzinfo für AI-Coding-Agenten

Dieses Repository ist aktuell sehr schlank (nur `README.md`). Die `README.md` enthält das konzeptionelle Ziel: ein modularisiertes, hochverfügbares, echtzeitnahes Raumfahrtsystem. Verwende die Datei als primären Kontext für Architekturentscheidungen und suche nach weiteren Quelltextdateien im Projekt, bevor Du Annahmen triffst.

**Schnellüberblick**
- **Branch**: `main` (Standardbranch)
- **Dev-Container**: Umgebung ist ein Dev-Container auf Ubuntu (siehe Workspace-Info). Terminal-Shell: `bash`.
- **Verfügbare CLI-Tools im Container**: `apt`, `dpkg`, `docker`, `git`, `gh`, `kubectl`, `curl`, `wget`, u.a.

**Ziele dieser Anleitung**
- Mach schnell produktive Änderungen, ohne unerwartete Architektur-Annahmen zu treffen.
- Documentiere referenzierbare Fundstellen (Dateiname + Pfad) in jeder Änderung.

**Vor dem Ändern**
- Führe eine Repo-Suche nach relevanten Dateien aus: `git ls-files` oder `rg --files`.
- Prüfe `README.md` für das Domänenmodell (GNC, FDIR, TT&C, etc.) und folge Terminologie daraus in Code/Kommentaren.

**Konkrete Arbeitsweisen für diesen Repo-Typ**
- Wenn neue Modules angelegt werden, nenne Ordner nach Subsystemen aus `README.md`, z. B. `gnc/`, `fdir/`, `ttc/`.
- Nutze deterministische, erklärbare APIs: Funktionen/Module sollten klar die Zustandsübergänge und Nachrichtenbus-Interfaces dokumentieren.
- Commit-Messages: Kurz-Typ-Präfix + Ursache, z. B. `feat(gnc): add feedforward controller module — implements EKF fusion`.

**Build / Tests / Debug (discoverable patterns)**
- Es gibt aktuell keine build-skripte oder tests im Repo. Wenn Du Tests hinzufügst, verwende Standard-Tools (`pytest` für Python, `npm`/`jest` für JS, `cmake`/`ctest` für C/C++). Dokumentiere die Run-Commands in `README.md`.
- Debugging im Devcontainer: Öffne Terminal und nutze `bash`; verwende `docker` / `gdb` / `strace` falls native Binaries vorhanden.

**Integration & Kommunikation**
- Das README beschreibt eine Lock-free Message Bus-Topologie. Wenn Du Implementierungen erstellst, füge eine `/docs`-Skizze des Nachrichtenformats (`JSON`/`protobuf`) hinzu und exemplarische `schema`-Dateien.

**Beispiele aus diesem Repo**
- Architektur-Quelle: `README.md` — verwende die dort aufgeführten Subsystemnamen als konventionelle Ordner-/Modulnamen.

**PR & Review Hinweise**
- Öffne Branches von `main` mit sprechendem Namen `feat/<subsystem>-short-desc`.
- Jede PR: kurze Zusammenfassung, Dateien die hinzugefügt/geändert wurden, Link zu relevanten Sektionen in `README.md`.

**Wenn Dateien fehlen / Unklarheiten**
- Frag den Maintainer nach erwarteter Sprache/Toolchain (z. B. Python vs C++). Dokumentiere Deine Frage in der PR-Beschreibung.

Wenn Du möchtest, kann ich jetzt Tests/CI-Skeleton hinzufügen oder ein erstes Modulgerüst nach `README.md` anlegen — sag mir welches Subsystem Du zuerst implementiert haben willst.

---
Bitte Feedback: Welche Bereiche fehlen oder sollen detaillierter sein? 
