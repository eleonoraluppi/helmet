import processing.serial.*;

// Serial Port Variables
Serial port; // Initialize Serial object
String buff = ""; // Create a serial buffer
int[] depths = new int[64]; // Create a list to parse serial into 
int[] velocities = new int[64]; // Create a list for velocity data
int[] data = new int[128];

// Shared Variables
int cols = 8; // Sensor is 8x8
int rows = 8;
int scale = 100; // Scale value for drawing our mesh

// Cursor Tracking Variables
float xPress = 0.0; // Store the cursor x position on mouse press 
float yPress = 0.0; // Store the cursor y position on mouse press

// Mesh Rotation Variables
float xRotOffset = 0; // Temporary x rotational offset (relevant during mouse drag)
float xRotPos = 0; // X rotational position
float zRotOffset = 0; // Temporary z rotational offset (relevant during mouse drag)
float zRotPos = 0; // Z rotational position
float scaleOffset = .5; // Scale factor from mouse wheel

// Main and Secondary Windows
DistanceWindow distanceWindow;
VelocityWindow velocityWindow;

void setup() {
  size(700, 700, P3D); // Main window for distance heatmap

  // Create secondary window
  distanceWindow = new DistanceWindow();
  velocityWindow = new VelocityWindow();

  port = new Serial(this, "COM4", 115200); // CHANGE COM4 TO YOUR SERIAL PORT
  port.bufferUntil(10); // ASCII LineFeed Character (\n)

  // Initialize data arrays
  for (int i = 0; i < 64; i++) {
    depths[i] = 0;
    velocities[i] = 0;
  }

  new Thread(() -> PApplet.runSketch(new String[] { "DistanceWindow" }, distanceWindow)).start();
  new Thread(() -> PApplet.runSketch(new String[] { "VelocityWindow" }, velocityWindow)).start();
}

void draw() {
  // The main draw loop handles only one of the windows
  background(0);
}

void serialEvent(Serial p) {
  buff = port.readString(); // Read the whole line
  buff = buff.substring(0, buff.length() - 1); // Remove the Carriage Return
  if (buff != "") 
  {
    data = int(split(buff, ',')); // Split at commas into our array

    for (int i = 0; i < 64; i++) {
      depths[i] = data[2 * i];
      velocities[i] = data[2 * i + 1];
    }
  }
}

// Window for Distance Heatmap
class DistanceWindow extends PApplet {
  float[][] terrain = new float[8][8];

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    surface.setTitle("Distance Heatmap");
  }

  public void draw() 
  {
    background(0);
    colorMode(HSB, 360, 255, 255);
    lights();
    noStroke();
    smooth();

    translate(width/2,height/2);
    rotateX(PI/3-(xRotOffset+xRotPos));
    rotateZ(0-zRotOffset-zRotPos);
    scale(scaleOffset);
    translate(-width/2, -height/2);

    for (int y = 0; y < rows; y++) 
    {
      for (int x = 0; x < cols; x++) 
      {
        terrain[x][y] = depths[x + y * cols] / 10;
      }
    }

    for (int y = 0; y < rows - 1; y++) {
      beginShape(TRIANGLE_STRIP);
      for (int x = 0; x < cols; x++) {
        fill(map(terrain[x][y],0,350,0,255),255,255);
        vertex(x * scale, y * scale, terrain[x][y]);
        vertex(x * scale, (y + 1) * scale, terrain[x][y + 1]);
      }
      endShape();
    }
  }

}

// Window for Velocity Heatmap
class VelocityWindow extends PApplet {
  float[][] terrain = new float[8][8];

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    surface.setTitle("Velocity Heatmap");
  }

  public void draw() {
    background(0);
    colorMode(HSB, 360, 255, 255);
    lights();
    noStroke();
    smooth();

    translate(width/2,height/2);
    rotateX(PI/3-(xRotOffset+xRotPos));
    rotateZ(0-zRotOffset-zRotPos);
    scale(scaleOffset);
    translate(-width/2, -height/2);

    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < cols; x++) {
        terrain[x][y] = velocities[x + y * cols] * 10.0;
      }
    }

    for (int y = 0; y < rows - 1; y++) 
    {
      beginShape(TRIANGLE_STRIP);
      for (int x = 0; x < cols; x++) 
      {
        float value = terrain[x][y];
        //fill(map(terrain[x][y],-150,150,0,255),255,255);
        fill(getHeatmapColor(value));

        vertex(x * scale, y * scale, terrain[x][y]);
        vertex(x * scale, (y + 1) * scale, terrain[x][y + 1]);
      }
      endShape();
    }
  }

  color getHeatmapColor(float value) 
  {
    float hue;
    if (value == 0) {
      hue = 240;
    } else if (value > 0) {
      if (value <= 75) {
        hue = map(value, 0, 75, 240, 180);
      } else {
        hue = map(value, 75, 150, 180, 0);
      }
    } else {
      if (value >= -75) {
        hue = map(value, -75, 0, 300, 240);
      } else {
        hue = map(value, -150, -75, 0, 300);
      }
    }
    return color(hue, 255, 255);
  }
}

// Between mouse drag events, draw will still be running, so we need to 
// update this offset at every event.
void mouseDragged() {
  xRotOffset = (mouseY-yPress)/100; 
  zRotOffset = (mouseX-xPress)/100; 
}

// To prevent the mesh position "snapping back" after releasing the mouse button
// We add the new temporary offset to the position and clear the temporary offset
// for the next mouse drag
void mouseReleased() {
  xRotPos += xRotOffset;
  xRotOffset = 0;
  zRotPos += zRotOffset;
  zRotOffset = 0;
}

// Whenever the mousewheel is scrolled, we adjust the scale offset of the mesh
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scaleOffset += e/10;
}
