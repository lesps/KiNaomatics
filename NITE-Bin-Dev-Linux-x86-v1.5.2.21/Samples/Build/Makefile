include Common/CommonDefs.mak

.PHONY: all

NETPROJ = 
ifneq "$(realpath /usr/bin/gmcs)" ""
	NETPROJ += Boxes.net
endif

all: $(NETPROJ) PointViewer Boxes.java SingleControl TrackPad SceneAnalysis Players Boxes PointServer CircleControl 


.PHONY: PointViewer
PointViewer:
	$(MAKE) -C ../PointViewer

.PHONY: Boxes.net
Boxes.net:
	$(MAKE) -C ../Boxes.net

.PHONY: Boxes.java
Boxes.java:
	$(MAKE) -C ../Boxes.java

.PHONY: SingleControl
SingleControl:
	$(MAKE) -C ../SingleControl

.PHONY: TrackPad
TrackPad:
	$(MAKE) -C ../TrackPad

.PHONY: SceneAnalysis
SceneAnalysis:
	$(MAKE) -C ../SceneAnalysis

.PHONY: Players
Players:
	$(MAKE) -C ../Players

.PHONY: Boxes
Boxes:
	$(MAKE) -C ../Boxes

.PHONY: PointServer
PointServer:
	$(MAKE) -C ../PointServer

.PHONY: CircleControl
CircleControl:
	$(MAKE) -C ../CircleControl
