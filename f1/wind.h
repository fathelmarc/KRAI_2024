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
    hadapRobot.setString("h = "+to_string(passT) + " p = " +to_string(pitch));
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

    sf::RectangleShape seedlingRack(sf::Vector2f(325,25));
    seedlingRack.setPosition(200,400-25);
    seedlingRack.setFillColor(sf::Color(128,128,128,120));

    sf::RectangleShape seedlingPlantingZone(sf::Vector2f(400,100));
    seedlingPlantingZone.setPosition(190.5,400-300);
    seedlingPlantingZone.setFillColor(sf::Color(128,128,128,120));
    
    sf::CircleShape take1(4.5);
    take1.setPosition(220.5,400 -(((4.5/2)+25)/2));
    take1.setFillColor(sf::Color::Red);
    sf::CircleShape take2(4.5);
    take2.setPosition(220.5+25,400 -(((4.5/2)+25)/2));
    take2.setFillColor(sf::Color::Red);
    sf::CircleShape take3(4.5);
    take3.setPosition(220.5+50,400 -(((4.5/2)+25)/2));
    take3.setFillColor(sf::Color::Red);
    sf::CircleShape take4(4.5);
    take4.setPosition(220.5+(25*3),400 -(((4.5/2)+25)/2));
    take4.setFillColor(sf::Color::Red);
    sf::CircleShape take5(4.5);
    take5.setPosition(220.5+(25*4),400 -(((4.5/2)+25)/2));
    take5.setFillColor(sf::Color::Red);
    sf::CircleShape take6(4.5);
    take6.setPosition(220.5+(25*5),400 -(((4.5/2)+25)/2));
    take6.setFillColor(sf::Color::Red);
    sf::CircleShape take7(4.5);
    take7.setPosition(220.5+(25*6),400 -(((4.5/2)+25)/2));
    take7.setFillColor(sf::Color::Red);
    sf::CircleShape take8(4.5);
    take8.setPosition(220.5+(25*7),400 -(((4.5/2)+25)/2));
    take8.setFillColor(sf::Color::Red);
    sf::CircleShape take9(4.5);
    take9.setPosition(220.5+(25*8),400 -(((4.5/2)+25)/2));
    take9.setFillColor(sf::Color::Red);
    sf::CircleShape take10(4.5);
    take10.setPosition(220.5+(25*9),400 -(((4.5/2)+25)/2));
    take10.setFillColor(sf::Color::Red);
    sf::CircleShape take11(4.5);
    take11.setPosition(220.5+(25*10),400 -(((4.5/2)+25)/2));
    take11.setFillColor(sf::Color::Red);
    sf::CircleShape take12(4.5);
    take12.setPosition(220.5+(25*11),400 -(((4.5/2)+25)/2));
    take12.setFillColor(sf::Color::Red);
    
    sf::CircleShape put1(7.5);
    put1.setPosition(230,400-272.5-10);
    put1.setFillColor(sf::Color::Red);
    sf::CircleShape put2(7.5);
    put2.setPosition(280,400-272.5-10);
    put2.setFillColor(sf::Color::Red);
    sf::CircleShape put3(7.5);
    put3.setPosition(230+(50*2),400-272.5-10);
    put3.setFillColor(sf::Color::Red);
    sf::CircleShape put4(7.5);
    put4.setPosition(230+(50*3),400-272.5-10);
    put4.setFillColor(sf::Color::Red);
    sf::CircleShape put5(7.5);
    put5.setPosition(230+(50*4),400-272.5-10);
    put5.setFillColor(sf::Color::Red);
    sf::CircleShape put6(7.5);
    put6.setPosition(230+(50*5),400-272.5-10);
    put6.setFillColor(sf::Color::Red);

    sf::CircleShape put7(7.5);
    put7.setPosition(230,400-222.5-10);
    put7.setFillColor(sf::Color::Red);
    sf::CircleShape put8(7.5);
    put8.setPosition(280,400-222.5-10);
    put8.setFillColor(sf::Color::Red);
    sf::CircleShape put9(7.5);
    put9.setPosition(230+(50*2),400-222.5-10);
    put9.setFillColor(sf::Color::Red);
    sf::CircleShape put10(7.5);
    put10.setPosition(230+(50*3),400-222.5-10);
    put10.setFillColor(sf::Color::Red);
    sf::CircleShape put11(7.5);
    put11.setPosition(230+(50*4),400-222.5-10);
    put11.setFillColor(sf::Color::Red);
    sf::CircleShape put12(7.5);
    put12.setPosition(230+(50*5),400-222.5-10);
    put12.setFillColor(sf::Color::Red);

    // window.draw(seedlingPlantingZone);
    window.draw(startPosition2);
    window.draw(startPosition);
    // window.draw(seedlingRack);
    // window.draw(take1);
    // window.draw(take2);
    // window.draw(take3);
    // window.draw(take4);
    // window.draw(take5);
    // window.draw(take6);
    // window.draw(take7);
    // window.draw(take8);
    // window.draw(take9);
    // window.draw(take10);
    // window.draw(take11);
    // window.draw(take12);

    // window.draw(put1);
    // window.draw(put2);
    // window.draw(put3);
    // window.draw(put4);
    // window.draw(put5);
    // window.draw(put6);
    // window.draw(put7);
    // window.draw(put8);
    // window.draw(put9);
    // window.draw(put10);
    // window.draw(put11);
    // window.draw(put12);
}

void lintasan(sf::RenderWindow& window){
    sf::VertexArray line1(sf::Lines, 5);
    line1[0].position = sf::Vector2f(125, 365); // Starting point of the line
    line1[1].position = sf::Vector2f(125, 365 - 205); // Ending point of the line
    line1[0].color = sf::Color::Green; // Set color
    line1[1].color = sf::Color::Green;


    sf::VertexArray line2(sf::Lines, 5);
    line2[0].position = sf::Vector2f(125 , 365 - 205); // Starting point of the line
    line2[1].position = sf::Vector2f(125 + 205, 365 - 205); // Ending point of the line
    line2[0].color = sf::Color::Green; // Set color
    line2[1].color = sf::Color::Green;

    sf::VertexArray line3(sf::Lines, 5);
    line3[0].position = sf::Vector2f(125 + 205, 365 - 205); // Starting point of the line
    line3[1].position = sf::Vector2f(125 + 205, 365); // Ending point of the line
    line3[0].color = sf::Color::Green; // Set color
    line3[1].color = sf::Color::Green;

    sf::VertexArray line4(sf::Lines, 5);
    line4[0].position = sf::Vector2f(125 + 205, 365); // Starting point of the line
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
    lintasan(window);
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
