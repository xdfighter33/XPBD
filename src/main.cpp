
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// 
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#define GLM_ENABLE_EXPERIMENTAL
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#endif
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Shader/shader_loader.h"
#include "Camera/camera.hpp"
#include "Model_loading/Models.hpp"
#include "Physics/SoftBodyPBD.hpp"
#include "Physics/SoftbodyXPBD.hpp"
#include <iostream>

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

bool capture = false;

// Paths towards shaders 
                    
std::string Model_fragmentShaderPath = std::string(SHADER_DIR) + "/model_frag.glsl";
std::string Model_VertexShaderPath   = std::string(SHADER_DIR) + "/model_vert.glsl";

std::string Shadow_Mapping_Frag_Path = std::string(SHADER_DIR) + "/shadow_map_frag.glsl";
std::string Shadow_Mapping_Vert_Path = std::string(SHADER_DIR) + "/shadow_map_vert.glsl";


std::string Phong_lightingPath = std::string(SHADER_DIR) + "/Phong_Lighting.glsl";

std::string CellShade_LightingPath = std::string(SHADER_DIR) + "/model_cellShading.glsl";


std::string Floor_VertexPath = std::string(SHADER_DIR) + "/floor_vert.glsl";
std::string Floor_FragPath = std::string(SHADER_DIR) + "/floor_frag.glsl";
// Path towards model
std::string Model_Spherepath = std::string(MODELS_DIR) + "/Sphere.obj";
std::string Model_CubePath = std::string(MODELS_DIR) + "/cube.obj";


glm::vec3 lightPos(5.0f, 10.0f, 5.0f);
glm::vec3 lightColor(1,1,1);
glm::vec3 objectColor(1.0,1.0,1.0); // Floor color please change the variable name 
glm::vec3 sphereColor(0.0f,0.0f,1.0f);
// Need view Position  


// Floor Information 
float floorVertices[] = {
     10.0f,  0.0f,  10.0f,  0.0f, 1.0f, 0.0f,  10.0f,  10.0f, // top right
     10.0f,  0.0f, -10.0f,  0.0f, 1.0f, 0.0f,  10.0f,   0.0f, // bottom right
    -10.0f,  0.0f, -10.0f,  0.0f, 1.0f, 0.0f,   0.0f,   0.0f, // bottom left
    -10.0f,  0.0f,  10.0f,  0.0f, 1.0f, 0.0f,   0.0f,  10.0f  // top left 
};

// Floor indices
unsigned int floorIndices[] = {
    0, 1, 3,  // first triangle
    1, 2, 3   // second triangle
};




void renderSoftBody(const SoftBodyPBD& softBody, Model& model, Shader& shader) {
    const auto& particles = softBody.getParticles();
    const auto& updatedNormals = softBody.getUpdatedNormals();

    // Update the model's vertex data
    for (size_t i = 0; i < particles.size(); ++i) {
        model.updateVertexPosition(i, particles[i].position);
        //   model.updateVertexNormal(i, updatedNormals[i]);
    }

    // Render the updated model
    model.Draw(shader);
}

void renderSoftBodyXPBD(const SoftBodyXPBD& softBody, Model& model, Shader& shader) {
    const auto& particles = softBody.getParticles();
    // const auto& updatedNormals = softBody.getUpdatedNormals();

    // Update the model's vertex data
    for (size_t i = 0; i < particles.size(); ++i) {
        model.updateVertexPosition(i, particles[i].position);
        //   model.updateVertexNormal(i, updatedNormals[i]);
    }

    // Render the updated model
    model.Draw(shader);
}




unsigned int floorVAO, floorVBO, floorEBO;
Shader* floorShader;
void setupFloor()
{
    glGenVertexArrays(1, &floorVAO);
    glGenBuffers(1, &floorVBO);
    glGenBuffers(1, &floorEBO);

    glBindVertexArray(floorVAO);

    glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floorVertices), floorVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, floorEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(floorIndices), floorIndices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(3);
    // normal attribute
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(4);
    // texture coord attribute
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(5);

    glBindVertexArray(0);

    floorShader = new Shader(Floor_VertexPath, Phong_lightingPath);
}

Shader* ShadowMapDepth;
const unsigned int SHADOW_HEIGHT = 2024, SHADOW_WIDTH = 2024;
unsigned int DepthMapFBO;
unsigned int depthMap;
void setupShadowMap() {
    // Create framebuffer object for rendering depth map
    glGenFramebuffers(1, &DepthMapFBO);

    // Create depth texture
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 
                 SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    // Attach depth texture as FBO's depth buffer
    glBindFramebuffer(GL_FRAMEBUFFER, DepthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Check if framebuffer is complete
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "Framebuffer not complete!" << std::endl;

    // Create and compile the depth map shader
    ShadowMapDepth = new Shader(Shadow_Mapping_Vert_Path, Shadow_Mapping_Frag_Path);
}

void renderScene(Shader* shader, SoftBodyXPBD& XPBD, Model& ourModel)
{
    // Render the soft body
    glm::mat4 model = glm::mat4(1.0f);
    shader->setMat4("model", model);
    renderSoftBodyXPBD(XPBD, ourModel, *shader);

    // Render the floor
    model = glm::mat4(1.0f);
    shader->setMat4("model", model);
    glBindVertexArray(floorVAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void renderSceneMain(Shader* shader,SoftBodyXPBD& XPBD,Model& ourModel)
{
    // Render the soft body
    glm::mat4 model = glm::mat4(1.0f);
    shader->setMat4("model", model);
    renderSoftBodyXPBD(XPBD, ourModel ,*shader);
    // Render the floor
    model = glm::mat4(1.0f);
    shader->setMat4("model", model);
    glBindVertexArray(floorVAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // If you have other objects in your scene that should cast shadows,
    // render them here as well
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window, SoftBodyXPBD& sim);



// settings
 unsigned int SCR_WIDTH = 1080;
 unsigned int SCR_HEIGHT = 1980;

// camera
Camera camera(glm::vec3(0.0f, 1.0f, 7.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "XPBD/PBD", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }











    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // tell stb_image.h to flip loaded texture's on the y-axis (before loading model).
    stbi_set_flip_vertically_on_load(true);

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);


    // ImGui: load all IMGUI funtions
    // ----------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
       io.Fonts->AddFontDefault();

        ImGui_ImplGlfw_InitForOpenGL(window, true);

        ImGui_ImplOpenGL3_Init(glsl_version);



    // build and compile shaders
    // -------------------------
    Shader ourShader(Model_VertexShaderPath, Phong_lightingPath);

    // load models
    // -----------
    Model ourModel(Model_Spherepath);

    // Set PBD
    // SoftBodyPBD SoftBody(ourModel);

    // Set XPBD
    float n = 120;
    float dt = 1.0f / n;// 60 fps 
    float it = 40;

    SoftBodyXPBD XPBD_SoftBody(ourModel,dt,it);
    // Load Floor 
    setupFloor();
    // draw in wireframe
     //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // SoftBody.setPosition(glm::vec3(0,2,0));
    XPBD_SoftBody.setPosition(glm::vec3(0,2,0));

    //  Setting up Window config

    bool window_instruments = true;

    // render loop
    // -----------

    float t = 0.5f;
    float shape = 0.01f;
    float shear = 1;
    float volume{0.2};
    float p_threshold(0.1f);
    float pCreep(0.1f);

    setupShadowMap();
    while (!glfwWindowShouldClose(window))
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        


                if(window_instruments)
        {       
            float its =  it;
   

            float fps = n;
            ImGui::Begin("Constraints!");                          // Create a window called "Hello, world!" and append into it.
            ImGui::Text("Constraints");
            ImGui::SliderFloat("Inverse Distance Constraint", &t,0.0f,1.0f);
            ImGui::SliderFloat("Volume Constraint ",&volume,0.0f,1.0f);
            ImGui::SliderFloat("Shear Constraint", &shear, 0.0f,1.0f);          
            ImGui::Text("Shape matching");
            ImGui::SliderFloat("Shape matching", &shape, 0.0f,1.0f);
            ImGui::SliderFloat("plastic creep", &pCreep, 0.0f, 1.0f);
            ImGui::SliderFloat("plastic threshold ", &p_threshold, 0.0f, 1.0f);
            ImGui::Text("System Settings");
            ImGui::SliderFloat("Iterations",&its,0.0f,100.0f);
            ImGui::SliderFloat("FPS",&fps,5,120);
            ImGui::Checkbox("Close window", &window_instruments);

            XPBD_SoftBody.SetDistaneConstraint(t);
            XPBD_SoftBody.set_vol(volume);
            XPBD_SoftBody.setShearingConstraints(shear);
            XPBD_SoftBody.setShapeFactor(shape);
            XPBD_SoftBody.setPlasticCreep(pCreep);
            XPBD_SoftBody.setPlasticThreshold(p_threshold);

            n = fps;
            it = its;
            
            ImGui::End();
        }


        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        //  SoftBody.update(deltaTime);
         XPBD_SoftBody.update();


        


        // input
        // -----
        processInput(window,XPBD_SoftBody);



    //Shadows 
    glm::mat4 lightProjection, lightView;
    glm::mat4 lightSpaceMatrix;
float near_plane = 1.0f, far_plane = 20.0f;
float ortho_size = 15.0f;
lightProjection = glm::ortho(-ortho_size, ortho_size, -ortho_size, ortho_size, near_plane, far_plane);
lightView = glm::lookAt(lightPos, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0, 1.0, 0.0));
lightSpaceMatrix = lightProjection * lightView;
    // renderSceneToDepthMap(XPBD_SoftBody, ourModel);
    



    ShadowMapDepth->use();
    ShadowMapDepth->setMat4("lightSpaceMatrix",lightSpaceMatrix);

            glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, DepthMapFBO);
            glClear(GL_DEPTH_BUFFER_BIT);
            renderScene(ShadowMapDepth,XPBD_SoftBody,ourModel);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, depthMap);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

                
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render
        // ------
       
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();

    floorShader->use();
    // lightPos.y += sin(glfwGetTime());
    // lightPos.z += sin(glfwGetTime());
    floorShader->setMat4("projection", projection);
    floorShader->setMat4("view", view);
    floorShader->setVec3("lightPos", lightPos);
    floorShader->setVec3("lightColor", lightColor);
    floorShader->setVec3("objectColor", objectColor);
    floorShader->setVec3("viewPos", camera.Position);
glActiveTexture(GL_TEXTURE1);
glBindTexture(GL_TEXTURE_2D, depthMap);
    floorShader->setInt("shadowMap", 1);
    floorShader->setMat4("lightSpaceMatrix",lightSpaceMatrix);
    glm::mat4 floor_model = glm::mat4(1.0f);
    floorShader->setMat4("model", floor_model);


    
    
    glBindVertexArray(floorVAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        // don't forget to enable shader before setting uniforms
   
        ourShader.use();

        // view/projection transformations
        ourShader.setInt("shadowMap", 1);
        ourShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        ourShader.setMat4("projection", projection);
        ourShader.setMat4("view", view);
        ourShader.setVec3("lightPos",lightPos);
        ourShader.setVec3("lightColor",lightColor);
        ourShader.setVec3("objectColor",sphereColor);
        ourShader.setVec3("viewPos",camera.Position);
        ourShader.setMat4("lightSpaceMatrix",lightSpaceMatrix);


        // render the loaded model
        glm::mat4 model = glm::mat4(1.0f);

        ourShader.setMat4("model", model);

        // Render objects 
        // renderSoftBody(SoftBody,ourModel,ourShader);
        // renderSoftBodyXPBD(XPBD_SoftBody,ourModel,ourShader);
        renderSceneMain(&ourShader,XPBD_SoftBody,ourModel);
        // ourModel.Draw(ourShader);

        // Render floor 
    


        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }



        //Delete ImGui resources
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();



    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glDeleteVertexArrays(1, &floorVAO);
    glDeleteBuffers(1, &floorVBO);
    glDeleteBuffers(1, &floorEBO);
    delete floorShader;
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window,SoftBodyXPBD& sim)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
        sim.resetToPosition(glm::vec3(0,10,0));
    if(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS){
        capture = true;
    }
    if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
        capture = false;
    }
    if(glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS){
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.

    SCR_WIDTH = width;
    SCR_HEIGHT = height;
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;
    if(capture){
     camera.ProcessMouseMovement(xoffset, yoffset);
    }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}