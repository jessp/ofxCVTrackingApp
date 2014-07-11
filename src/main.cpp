#include "testApp.h"
#include "ofAppGLFWWindow.h"

int main() {
    
    ofAppGLFWWindow* window = new ofAppGLFWWindow();
//    window->setMultiDisplayFullscreen(true);
	ofSetupOpenGL(window, 1000, 1000, OF_WINDOW);
	ofRunApp(new testApp());
}
