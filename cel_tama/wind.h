#include <SFML/Graphics.hpp>
#include "fethernet.h"
#include "control.h"
sf::RenderWindow window(sf::VideoMode(850, 400), "Real-Time Robot Position Graph", sf::Style::Close);
sf::VertexArray trail(sf::LineStrip); // Jejak robot

void text(sf::RenderWindow& window,float x,float y){
    inCaseAuto();
    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")) {
        cerr << "Gagal memuat font." << endl;
        return;
    }
    sf::Text posisi;
    posisi.setFont(font);
    posisi.setCharacterSize(10);
    posisi.setFillColor(sf::Color::White);
    posisi.setString("caseBot : " + to_string(caseBot));
    posisi.setPosition(590, 55.f+15.f);

    sf::Text prevPosisi;
    prevPosisi.setFont(font);
    prevPosisi.setCharacterSize(10);
    prevPosisi.setFillColor(sf::Color::White);
    prevPosisi.setString("prevcase : " + to_string(prevcase));
    prevPosisi.setPosition(590, 55.f+30.f);
    // Buat objek teks untuk menampilkan posisi robot
    sf::Text posisiRobot;
    posisiRobot.setFont(font);
    posisiRobot.setCharacterSize(10);
    posisiRobot.setFillColor(sf::Color::White);
    posisiRobot.setString("PosisiRobot: x = " + to_string(x) + ", y = " + to_string((y*1)));
    // Posisi teks posisi robot tetap di kiri atas
    posisiRobot.setPosition(590, 10.f);
    sf::Text hadapRobot;
    hadapRobot.setFont(font);
    hadapRobot.setCharacterSize(10);
    hadapRobot.setFillColor(sf::Color::White);
    hadapRobot.setString("h = "+to_string(yaw) + " p = " +to_string(pitch) +" r = " + to_string(roll));
    hadapRobot.setPosition(590, 10.f+15);
    sf::Text bridge;
    bridge.setFont(font);
    bridge.setCharacterSize(10);
    bridge.setFillColor(sf::Color::White);
    bridge.setString("tanjakkan = "+to_string(tanjakkan));
    // Posisi teks posisi robot tetap di kiri atas
    bridge.setPosition(590, 10.f+15 + 15);
    // Gambar teks lokasi robot di jendela
    window.draw(bridge);
    window.draw(posisiRobot);
    window.draw(hadapRobot);
    window.draw(posisi);
    window.draw(prevPosisi);
}
void lapangan(sf::RenderWindow& window){
    sf::RectangleShape startPosition(sf::Vector2f(70.f,70.f));
    startPosition.setPosition(90,330);
    startPosition.setFillColor(sf::Color::Red);

    sf::RectangleShape startPosition2(sf::Vector2f(70.f,70.f));
    startPosition2.setPosition(15,330);
    startPosition2.setFillColor(sf::Color::Red);

    // window.draw(seedlingPlantingZone);
    window.draw(startPosition2);
    window.draw(startPosition);
}

void lintasan(sf::RenderWindow& window){
    sf::VertexArray line1(sf::Lines, 5);
    line1[0].position = sf::Vector2f(125, 365); // Starting point of the line
    line1[1].position = sf::Vector2f(125, 265); // Ending point of the line
    line1[0].color = sf::Color::Green; // Set color
    line1[1].color = sf::Color::Green;


    sf::VertexArray line2(sf::Lines, 5);
    line2[0].position = sf::Vector2f(125 , 265); // Starting point of the line
    line2[1].position = sf::Vector2f(125 + 100, 265); // Ending point of the line
    line2[0].color = sf::Color::Green; // Set color
    line2[1].color = sf::Color::Green;

    sf::VertexArray line3(sf::Lines, 5);
    line3[0].position = sf::Vector2f(125 + 100, 265); // Starting point of the line
    line3[1].position = sf::Vector2f(125 + 100, 365); // Ending point of the line
    line3[0].color = sf::Color::Green; // Set color
    line3[1].color = sf::Color::Green;

    sf::VertexArray line4(sf::Lines, 5);
    line4[0].position = sf::Vector2f(125 + 100, 365); // Starting point of the line
    line4[1].position = sf::Vector2f(125, 365); // Ending point of the line
    line4[0].color = sf::Color::Green; // Set color
    line4[1].color = sf::Color::Green;
    // Draw the line
    window.draw(line1);
    window.draw(line2);
    window.draw(line3);
    window.draw(line4);
}
// Fungsi untuk menampilkan posisi robot dan teksnya
void displayPosition(sf::RenderWindow& window,sf::VertexArray& trail, float x, float y) {
    sf::RectangleShape tama(sf::Vector2f(50.f, 50.f)); // Ubah ukuran titik sesuai kebutuhan
    float scale = 10;
    tama.setPosition(100 + x*scale , 340-y*scale);
    tama.setFillColor(sf::Color(0, 0, 255, 128));
    trail.append(sf::Vertex(sf::Vector2f(tama.getPosition().x +25, tama.getPosition().y + 25), sf::Color::White));

    sf::VertexArray pembatas(sf::Lines, 2);
    pembatas[0].position = sf::Vector2f(587, 0); // Starting point of the pembatas
    pembatas[1].position = sf::Vector2f(587, 400); // Ending point of the pembatas
    pembatas[0].color = sf::Color::Red; // Set color
    pembatas[1].color = sf::Color::Red;
    // Bersihkan jendela dengan warna putih
    window.clear(sf::Color::Black); 
    
    lapangan(window);
    // Gambar jejak di jendela
    window.draw(trail);
    text(window,x,y);
    window.draw(tama);
    // lintasan(window);
    window.draw(pembatas);
    // Gambar garis panduan di jendela
    // window.draw(guideline);    
    // Gambar titik (posisi robot) di jendela
   
   
    // Tampilkan isi jendela
    window.display();
}
void check(sf::RenderWindow& window){
    sf::Event event;
    while (window.pollEvent(event)) {
        // Handle events
        if (event.type == sf::Event::Closed) {
            window.close();
        }
    }
}
