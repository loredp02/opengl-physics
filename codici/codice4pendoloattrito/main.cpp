#include <iostream>
#include <sstream>
#include "GL/glew.h" // prima di freeglut
#include "GL/freeglut.h"
#include "glm/glm.hpp"

#include "transform.h"
#include "camera.h"
#include "shaderclass.h"
#include "myshaderclass.h"

#include "texture2D.h"

// Includiamo le librerie per bullet
#include "btBulletDynamicsCommon.h"
#include <stdio.h>

/**
  Struttura di comodo dove sono memorizzate tutte le variabili globali
*/
struct global_struct {

  int WINDOW_WIDTH  = 1024; // Larghezza della finestra 
  int WINDOW_HEIGHT = 768; // Altezza della finestra

  GLuint VAO; // Vertex Array Object

  Camera camera;

  // Istanza della classe di gestione degli shader 
  MyShaderClass shaders;

  AmbientLight     ambient_light;
  DirectionalLight directional_light;
  DiffusiveLight   diffusive_light;
  SpecularLight    specular_light;

  Texture2D cube_texture; // Oggetto texture

  const float SPEED = 2;
  float gradX;
  float gradY; 

  global_struct() : gradX(0.0f), gradY(0.0f) {}

  float deltat = 1.0f / 60.0f; // frequenza di aggiornamento dell'animazione (60 fps)
  const float g = -9; //accelerazione di gravità

} global;

/**
  Struttura dati che contiene gli attributi di un vertice.
*/
struct Vertex {
  glm::vec3 position; ///< Coordinate spaziali
  glm::vec3 color;    ///< Colore
  glm::vec3 normal;   ///< Normale
  glm::vec2 textcoord;///< Coordinate di texture

  Vertex(
      float x, float y, float z, 
      float r, float g, float b, 
      float xn, float yn, float zn,
      float s, float t) {
      
      position = glm::vec3(x,y,z);
      color = glm::vec3(r,g,b);
      normal = glm::vec3(xn,yn,zn);
      textcoord = glm::vec2(s,t);
  }

  Vertex(const glm::vec3 &xyz, const glm::vec3 &rgb, const glm::vec3 &norm, const glm::vec2 &txt) 
  : position(xyz), color(rgb), normal(norm), textcoord(txt) {}
};

// Creiamo una struttura per centralizzare la Fisica di Bullet
struct Physics {
  btDiscreteDynamicsWorld* dynamicsWorld; //andiamo a creare il mondo dinamico
  //per la realizzazione della collision detection phase
  btBroadphaseInterface* broadphase; 
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  //per risolvere l'urto
  btSequentialImpulseConstraintSolver* solver;

  //costruttore
  Physics() : dynamicsWorld(nullptr), broadphase(nullptr), collisionConfiguration(nullptr), dispatcher(nullptr), solver(nullptr) {}

  //distruttore
  ~Physics() {
        delete dynamicsWorld;
        delete solver;
        delete dispatcher;
        delete collisionConfiguration;
        delete broadphase;
  }
} physics;

/**
Prototipi della nostre funzioni di callback. 
Sono definite più avanti nel codice.
*/
void MyRenderScene(void);
void MyIdle(void);
void MyKeyboard(unsigned char key, int x, int y);
void MyClose(void);
void MySpecialKeyboard(int Key, int x, int y);
void MyMouse(int x, int y);

//Creazione di una funzione che inizializza la struct Physics, andremo a chiamarlo nell'init.
void init_physics(){
  //inizializziamo i campi della struct Physics
  physics.broadphase = new btDbvtBroadphase(); //il metodo Dbvt configura la broadphase usando AABB con HBV (hierarchy bounding volume)
  physics.collisionConfiguration  = new btDefaultCollisionConfiguration(); //https://pybullet.org/Bullet/BulletFull/classbtDefaultCollisionConfiguration.html#details
  physics.dispatcher = new btCollisionDispatcher(physics.collisionConfiguration); //gestione delle collisioni a partire da CollisionConfiguration che gestisce l'allocazione della memoria nella collisione
  physics.solver = new btSequentialImpulseConstraintSolver; //il risolutore della collisione fa uso dell'approccio ad impulsi sequenziali
  physics.dynamicsWorld = new btDiscreteDynamicsWorld(physics.dispatcher, physics.broadphase, 
                                                        physics.solver, physics.collisionConfiguration); //creazione del mondo dinamico

  

  physics.dynamicsWorld->setGravity(btVector3(0,global.g, 0)); //impostiamo la gravità del mondo con un vettore tridimensionale con ogni valore del vettore l'accelerazione nella direzione corrispondente

}

//Creazione di una funzione per creare un corpo rigido
btRigidBody* createRB(float massa, const btVector3& pos, btCollisionShape* forma) {
  btTransform startTransform;
  // settiamo il corpo all'origine del mondo
  startTransform.setIdentity();
  startTransform.setOrigin(pos);

  btVector3 Inertia(0,0,0);
  if (massa != 0.f) //la condizione di massa uguale a zero corrisponde ad un body detto "static", ad esempio il pavimento della scena o un muro.
  {
    forma->calculateLocalInertia(massa, Inertia); 
  }

  btDefaultMotionState* motionState = new btDefaultMotionState(startTransform); // lo stato del moto iniziale è quello di startTransform
  btRigidBody::btRigidBodyConstructionInfo Info(massa, motionState, forma, Inertia); //il motionState servirà a sincronizzare l'oggetto fisico con quello grafico (la mesh)
  btRigidBody* body = new btRigidBody(Info); //crea il corpo rigido a partire dalle informazioni

  physics.dynamicsWorld->addRigidBody(body); //aggiunge il nuovo corpo rigido al mondo dinamico

  return body;
}

// A questo punto si può creare un corpo rigido cubico corrispondente a ciò che abbiamo in scena.
btRigidBody* cubeRB = nullptr; //memorizziamo un puntatore al cubo che verrà creato
void create_physics_cube() {
  btCollisionShape* cubeShape = new btBoxShape(btVector3(1,1,1)); 

  float massaCube = 1.0;

  cubeRB = createRB(1.0f, btVector3(0,0,0), cubeShape); //leghiamo il puntatore al corpo rigido
}

void create_physics_pendulum(){
// Definiamo la posizione dell'ancoraggio del pendolo
btVector3 anchorPoint(5,5,0);

btVector3 anchorInCube = cubeRB->getCenterOfMassTransform().inverse() * anchorPoint;

  // Definiamo il vincolo punto-punto tra il cubo e l'ancoraggio
  btPoint2PointConstraint* pendulumConstraint = new btPoint2PointConstraint(
    *cubeRB,  
    anchorInCube
  );

  //Aggiungiamo il vincolo al mondo fisico
  physics.dynamicsWorld->addConstraint(pendulumConstraint, true);

  //Aggiungiamo uno smorzamento al pendolo creando un effetto di damping lineare e angolare 
  cubeRB->setDamping(0.05f, 0.05f);

}

// Funzione per aggiornare il mondo fisico e farlo avanzare di uno StepSimulation
void update_phys(){
  physics.dynamicsWorld->stepSimulation(global.deltat, 10); //facciamo 10 substeps
}

// Funzione che aggiorna continuamente la scena per creare l'animazione
void update(int value){
  update_phys();

  glutPostRedisplay();

  glutTimerFunc(16, update, 0);
}

// Funzione per sincronizzare il corpo fisico con il modello
void sync_btgl(){
  btTransform trans;
  cubeRB->getMotionState()->getWorldTransform(trans); //grazie al motionstate riusciamo a recuperare la trasformazione come matrice 4x4

  float matrix[16];
  trans.getOpenGLMatrix(matrix); //formattiamo la matrice per openGL

  //creiamo una matrice di GLM con la matrice appena creata.
  glm::mat4 trans_modello = glm::mat4(
            matrix[0],  matrix[1],  matrix[2],  matrix[3],
            matrix[4],  matrix[5],  matrix[6],  matrix[7],
            matrix[8],  matrix[9],  matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15]
        );

  global.shaders.set_model_transform(trans_modello); //applichiamo la matrice al corpo

}

void init(int argc, char*argv[]) {
  
  //inizializziamo la fisica
  init_physics();

  

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);


  glutInitWindowSize(global.WINDOW_WIDTH, global.WINDOW_HEIGHT);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Informatica Grafica");

  glutSetCursor(GLUT_CURSOR_NONE);

  // Portiamo il mouse fisso al centro della finestra
  global.camera.set_mouse_init_position(global.WINDOW_WIDTH/2, global.WINDOW_HEIGHT/2);
  global.camera.lock_mouse_position(true);
  glutWarpPointer(global.WINDOW_WIDTH/2, global.WINDOW_HEIGHT/2);

 // Must be done after glut is initialized!
  GLenum res = glewInit();
  if (res != GLEW_OK) {
      std::cerr<<"Error : "<<glewGetErrorString(res)<<std::endl;
    exit(1);
  }

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  glutDisplayFunc(MyRenderScene);

  glutKeyboardFunc(MyKeyboard);

  glutCloseFunc(MyClose);

  glutSpecialFunc(MySpecialKeyboard);

  // Callback per la gesione degli eventi del mouse.
  glutPassiveMotionFunc(MyMouse);


  glEnable(GL_DEPTH_TEST);

  
}

void create_scene() {
  Vertex Vertices[36] = {
        Vertex(glm::vec3(-1.0f,-1.0f, 1.0f), glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0,0,1), glm::vec2(0,0)),
        Vertex(glm::vec3( 1.0f,-1.0f, 1.0f), glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0,0,1), glm::vec2(1,0)),
        Vertex(glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0,0,1), glm::vec2(0,1)),
        Vertex(glm::vec3( 1.0f,-1.0f, 1.0f), glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0,0,1), glm::vec2(1,0)),
        Vertex(glm::vec3( 1.0f, 1.0f, 1.0f), glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0,0,1), glm::vec2(1,1)),
        Vertex(glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0,0,1), glm::vec2(0,1)),


        Vertex(glm::vec3( 1.0f,-1.0f, 1.0f), glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(1,0,0), glm::vec2(0,0)),
        Vertex(glm::vec3( 1.0f,-1.0f,-1.0f), glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(1,0,0), glm::vec2(1,0)),
        Vertex(glm::vec3( 1.0f, 1.0f, 1.0f), glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(1,0,0), glm::vec2(0,1)),
        Vertex(glm::vec3( 1.0f,-1.0f,-1.0f), glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(1,0,0), glm::vec2(1,0)),
        Vertex(glm::vec3( 1.0f, 1.0f,-1.0f), glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(1,0,0), glm::vec2(1,1)),
        Vertex(glm::vec3( 1.0f, 1.0f, 1.0f), glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(1,0,0), glm::vec2(0,1)),


        Vertex(glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0,1,0), glm::vec2(0,0)),
        Vertex(glm::vec3( 1.0f, 1.0f, 1.0f), glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0,1,0), glm::vec2(1,0)),
        Vertex(glm::vec3(-1.0f, 1.0f,-1.0f), glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0,1,0), glm::vec2(0,1)),
        Vertex(glm::vec3( 1.0f, 1.0f, 1.0f), glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0,1,0), glm::vec2(1,0)),
        Vertex(glm::vec3( 1.0f, 1.0f,-1.0f), glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0,1,0), glm::vec2(1,1)),
        Vertex(glm::vec3(-1.0f, 1.0f,-1.0f), glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0,1,0), glm::vec2(0,1)),


        Vertex(glm::vec3(-1.0f,-1.0f, 1.0f), glm::vec3( 1.0f, 1.0f, 0.0f), glm::vec3(-1,0,0), glm::vec2(1,0)),
        Vertex(glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3( 1.0f, 1.0f, 0.0f), glm::vec3(-1,0,0), glm::vec2(1,1)),
        Vertex(glm::vec3(-1.0f,-1.0f,-1.0f), glm::vec3( 1.0f, 1.0f, 0.0f), glm::vec3(-1,0,0), glm::vec2(0,0)),
        Vertex(glm::vec3(-1.0f,-1.0f,-1.0f), glm::vec3( 1.0f, 1.0f, 0.0f), glm::vec3(-1,0,0), glm::vec2(0,0)),
        Vertex(glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3( 1.0f, 1.0f, 0.0f), glm::vec3(-1,0,0), glm::vec2(1,1)),
        Vertex(glm::vec3(-1.0f, 1.0f,-1.0f), glm::vec3( 1.0f, 1.0f, 0.0f), glm::vec3(-1,0,0), glm::vec2(0,1)),


        Vertex(glm::vec3(-1.0f,-1.0f, 1.0f), glm::vec3( 0.0f, 1.0f, 1.0f), glm::vec3(0,-1,0), glm::vec2(0,1)),
        Vertex(glm::vec3(-1.0f,-1.0f,-1.0f), glm::vec3( 0.0f, 1.0f, 1.0f), glm::vec3(0,-1,0), glm::vec2(0,0)),
        Vertex(glm::vec3( 1.0f,-1.0f, 1.0f), glm::vec3( 0.0f, 1.0f, 1.0f), glm::vec3(0,-1,0), glm::vec2(1,1)),
        Vertex(glm::vec3( 1.0f,-1.0f, 1.0f), glm::vec3( 0.0f, 1.0f, 1.0f), glm::vec3(0,-1,0), glm::vec2(1,1)),
        Vertex(glm::vec3(-1.0f,-1.0f,-1.0f), glm::vec3( 0.0f, 1.0f, 1.0f), glm::vec3(0,-1,0), glm::vec2(0,0)),
        Vertex(glm::vec3( 1.0f,-1.0f,-1.0f), glm::vec3( 0.0f, 1.0f, 1.0f), glm::vec3(0,-1,0), glm::vec2(1,0)),


        Vertex(glm::vec3(-1.0f,-1.0f,-1.0f), glm::vec3( 1.0f, 0.0f, 1.0f), glm::vec3(0,0,-1), glm::vec2(1,0)),
        Vertex(glm::vec3(-1.0f, 1.0f,-1.0f), glm::vec3( 1.0f, 0.0f, 1.0f), glm::vec3(0,0,-1), glm::vec2(1,1)),
        Vertex(glm::vec3( 1.0f,-1.0f,-1.0f), glm::vec3( 1.0f, 0.0f, 1.0f), glm::vec3(0,0,-1), glm::vec2(0,0)),
        Vertex(glm::vec3( 1.0f,-1.0f,-1.0f), glm::vec3( 1.0f, 0.0f, 1.0f), glm::vec3(0,0,-1), glm::vec2(0,0)),
        Vertex(glm::vec3(-1.0f, 1.0f,-1.0f), glm::vec3( 1.0f, 0.0f, 1.0f), glm::vec3(0,0,-1), glm::vec2(1,1)),
        Vertex(glm::vec3( 1.0f, 1.0f,-1.0f), glm::vec3( 1.0f, 0.0f, 1.0f), glm::vec3(0,0,-1), glm::vec2(0,1))

  };
  
  glGenVertexArrays(1, &(global.VAO));
  glBindVertexArray(global.VAO);
 
  GLuint VBO;
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
    reinterpret_cast<GLvoid*>(offsetof(struct Vertex, position)));
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
    reinterpret_cast<GLvoid*>(offsetof(struct Vertex, color)));
  glEnableVertexAttribArray(1);

  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
    reinterpret_cast<GLvoid*>(offsetof(struct Vertex, normal)));
  glEnableVertexAttribArray(2);

  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
    reinterpret_cast<GLvoid*>(offsetof(struct Vertex, textcoord)));
  glEnableVertexAttribArray(3);


  global.camera.set_camera(
    glm::vec3(0,0,13), //pos
    glm::vec3(5,0,0), //look at
    glm::vec3(0,1,0)
  );


  global.camera.set_perspective(
    45.0f,
    global.WINDOW_WIDTH,
    global.WINDOW_HEIGHT,
    0.1,
    100
  );

  // Inizializza la classe degli shaders
  // Vengono caricati gli shader indicati nel metodo init e 
  // automaticamente linkato il programma 
  if (!global.shaders.init()) {
    std::cerr << "Error initializing shaders..." << std::endl;
    exit(0);
  }


  global.shaders.enable();

  global.ambient_light = AmbientLight(glm::vec3(1,1,1),0.2);

  global.directional_light = DirectionalLight(glm::vec3(1,1,1),glm::vec3(0,0,-1));

  global.diffusive_light = DiffusiveLight();

  global.specular_light = SpecularLight(1,30);


  // TEXTURE

  // Carichiamo in memoria la texture
  global.cube_texture.load("face.png");

  // Attiviamo la texture facendone un binding alla TextureUnit 0 (GL_TEXTURE0) 
  global.cube_texture.bind(0);

  // Settiamo l'oggetto ColorTextSampler nel fragment shader affinchè sia associato
  // alla TextureUnit 0
  global.shaders.set_color_texture_sampler(0);

  // creiamo il cubo fisico
  create_physics_cube();
  create_physics_pendulum();
}

void MyRenderScene() {
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  //la matrice di trasformazione la otterremo da Bullet
  update_phys();

  //sincronizziamo la trasformazione di bullet col disegno del cubo grafico
  sync_btgl();

  //a prescindere dal cubo le altre trasformazioni rimangono uguali, il global.shaders.set_ per il cubo è già nella funzione sync_btgl()
  global.shaders.set_camera_transform(global.camera.CP());
  global.shaders.set_ambient_light(global.ambient_light);
  global.shaders.set_directional_light(global.directional_light);
  global.shaders.set_diffusive_light(global.diffusive_light);
  global.shaders.set_specular_light(global.specular_light);
  global.shaders.set_camera_position(global.camera.position());



  //disegno della scena grafica
  glBindVertexArray(global.VAO);

  glDrawArrays(GL_TRIANGLES, 0, 36);

  glBindVertexArray(0);
  
  glutSwapBuffers();

}

// Funzione globale che si occupa di gestire l'input da tastiera.
void MyKeyboard(unsigned char key, int x, int y) {
  switch ( key )
  {
    case 27: // Escape key
      glutDestroyWindow(glutGetWindow());
      return;
    break;

    case 'a':
      global.gradY -= global.SPEED;
    break;
    case 'd':
      global.gradY += global.SPEED;
    break;
    case 'w':
      global.gradX -= global.SPEED;
    break;
    case 's':
      global.gradX += global.SPEED;
    break;

    // Variamo l'intensità di luce ambientale
    case '1':
      global.ambient_light.dec(0.05);
    break;

    // Variamo l'intensità di luce ambientale
    case '2':
      global.ambient_light.inc(0.05);
    break;

    // Variamo l'intensità di luce diffusiva
    case '3':
      global.diffusive_light.dec(0.05);
    break;

    // Variamo l'intensità di luce diffusiva
    case '4':
      global.diffusive_light.inc(0.05);
    break;

    // Variamo l'intensità di luce speculare
    case '5':
      global.specular_light.dec(0.05);
    break;

    // Variamo l'intensità di luce speculare
    case '6':
      global.specular_light.inc(0.05);
    break;

    // Variamo l'esponente della luce speculare
    case '7':
      global.specular_light.dec_shine(1);
    break;

    // Variamo l'esponente della luce speculare
    case '8':
      global.specular_light.inc_shine(1);
    break;

    case ' ': // Reimpostiamo la camera
      global.camera.set_camera(
        glm::vec3(0,0,4),
        glm::vec3(0,0,0),
        glm::vec3(0,1,0)
      );
      global.gradX = global.gradY = 0;
    break;
  }

  glutPostRedisplay();
}

void MySpecialKeyboard(int Key, int x, int y) {
  global.camera.onSpecialKeyboard(Key);
  glutPostRedisplay();
}

void MyMouse(int x, int y) {
  if (global.camera.onMouse(x,y)) {
    // Risposto il mouse al centro della finestra
    glutWarpPointer(global.WINDOW_WIDTH/2, global.WINDOW_HEIGHT/2);
  }
  glutPostRedisplay();
}


// Funzione globale che si occupa di gestire la chiusura della finestra.
void MyClose(void) {
  std::cout << "Tearing down the system..." << std::endl;
  // si rimuovono i corpi rigidi dal mondo dinamico
  for (int i = physics.dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = physics.dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        physics.dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

  // si rimuovono le strutture del mondo dinamico
  delete physics.dynamicsWorld;
  delete physics.solver;
  delete physics.broadphase;
  delete physics.dispatcher;
  delete physics.collisionConfiguration;
    

  // A schermo intero dobbiamo uccidere l'applicazione.
  exit(0);
}

int main(int argc, char* argv[])
{
  init(argc,argv);

  create_scene();

  glutTimerFunc(0, update, 0);

  glutMainLoop();
  
  return 0;
}