//RPG
let xp = 0;  //Var- variable
let hp = 100;  //;- optional, end of line
let gold = 50;
let currentWeapon = 0;  //Lower camelcase- convention
//let- better (modern JS, fixes a number of unusual behaviors... difficult to reason...)
//const- never changes...
//var- allows the most changing... more bugs
//Highlight then ctrl+D- highlight next...

let fighting;  //No initialization
let monsterHP;
let inventory = ["stick"];  //'' and back ticks? also work
// let inventory = ["stick", "dagger", "sword"];  //Array- can store any data type including objects
//Variables at start- good practice

//To update HTML elements on a webpage- References in JS code
//E.g. let el = document.querySelector("#el");
const button1 = document.querySelector("#button1");  //#button1- id...
const button2 = document.querySelector("#button2");
const button3 = document.querySelector("#button3");
const text = document.querySelector("#text");
const xpText = document.querySelector("#xpText");
const healthText = document.querySelector("#healthText");
const goldText = document.querySelector("#goldText");
const monsterStats = document.querySelector("#monsterStats");
const monsterName = document.querySelector("#monsterName");
const monsterHealth = document.querySelector("#monsterHealth");

const dragon = document.querySelector("#dragon");

const weapons = [  //Spacing- more readable
    {
        name: "stick",
        power: 5
    }, 
    {
        name: "dagger", 
        power: 30
    }, 
    {
        name: "claw hammer", 
        power: 50
    }, 
    {
        name: "sword", 
        power: 100
    }
];

const monsters = [
    {
        name: "slime", 
        level: 2, 
        health: 15, 
    }, 
    {
        name: "fanged beast", 
        level: 8, 
        health: 60, 
    }, 
    {
        name: "dragon", 
        level: 20, 
        health: 300, 
    }, 
]

const locations = [
    {
        name: "town square",  //""- since 2 words
        "button text": ["Go to store", "Go to cave", "Fight dragon"], 
        "button functions": [goStore, goCave, fightDragon], 
        text: "You are in the town square. You see a sign that says \"Store.\""  //\- Escaping
    }, 
    {
        name: "store", 
        "button text": ["Buy 10 health (10 gold)", 
            "Buy weapon (30 gold)", 
            "Go to Town Square"], 
        "button functions": [buyHealth, buyWeapon, goTown], 
        text: "You are in the store."
    }, 
    {
        name: "cave", 
        "button text": ["Fight slime", 
            "Fight fang beast", 
            "Go to Town Square"], 
        "button functions": [fightSlime, fightBeast, goTown], 
        text: "You are in the cave. You see some monsters."
    }, 
    {
        name: "fight", 
        "button text": ["Attack", 
            "Dodge", 
            "Run"], 
        "button functions": [attack, dodge, goTown], 
        text: "You are fighting a monster."
    }, 
    {
        name: "kill monster", 
        "button text": ["Go to Town Square", 
            "Go to Town Square", 
            "Go to Town Square"], 
        "button functions": [goTown, goTown, easterEgg], 
        text: 'The monster screams "Argh!" as it dies. You gain gold and experience.'  //''
    }, 
    {
        name: "lose", 
        "button text": ["Replay?", 
            "Replay?", 
            "Replay?"], 
        "button functions": [restart, restart, restart], 
        text: "You die."
    }, 
    {
        name: "win", 
        "button text": ["Replay?", 
            "Replay?", 
            "Replay?"], 
        "button functions": [restart, restart, restart], 
        text: "You defeat the dragon! You win the game!"
    }, 
    {
        name: "easter egg", 
        "button text": ["2", 
            "8", 
            "Go to Town Square"], 
        "button functions": [pick2, pick8, goTown], 
        text: "You found a secret game. Pick a number above. Ten numbers will be randomly chosen between 0 and 10. If the number you choose matches one of the random numbers, you win!"
    }
];
//Objects- access/modify their data via properties (instead of indices)
//{}- empty objects, object- consists of key/name-value pairs?

//Comment
/*
Comment
*/

//Initialize buttons
// button.onclick = openProgram;  //openProgram is a function?
button1.onclick = goStore;
button2.onclick = goCave;
button3.onclick = fightDragon;

//Create functions, indent- optional but adds readability
//If you have repetitive code- sign for a new function
function update(location) {  //location is a parameter
    monsterStats.style.display = "none";

    button1.innerText = location["button text"][0];
    button2.innerText = location["button text"][1];
    button3.innerText = location["button text"][2];

    button1.onclick = location["button functions"][0];
    button2.onclick = location["button functions"][1];
    button3.onclick = location["button functions"][2];

    // text.innerText = location["text"];
    text.innerText = location.text;  //Only works for single words
}

function goTown() {
    update(locations[0]);
}

function goStore() {  //Outputs message "Going to store." to the web console
    // console.log("Going to store.");
    // button1.innerText = "Buy 10 health (10 gold)";
    // button2.innerText = "Buy weapon (30 gold)";
    // button3.innerText = "Go to Town Square";  //Go back

    // button1.onclick = buyHealth;
    // button2.onclick = buyWeapon;
    // button3.onclick = goTown;

    // text.innerText = "You are in the store.";

    update(locations[1]);
}

function buyHealth() {
    //Conditionals
    if (gold >= 10) {
        hp = hp + 10;
        gold -= 10;  //Compound assignment
        healthText.innerText = hp;
        goldText.innerText = gold;
    }
    else {
        text.innerText = "You do not have enough gold to buy health.";
    }
}

function buyWeapon() {
    if (currentWeapon < (weapons.length - 1)) {
        if (gold >= 30) {
            gold -= 30;
            currentWeapon++;  //Another shortcut, +1
            goldText.innerText = gold;
            let newWeapon = weapons[currentWeapon].name;
            text.innerText = "You now have a new weapon. You now have a " + newWeapon + ".";
            inventory.push(newWeapon);
            text.innerText += "\n\nIn your inventory you have: " + inventory + ".";
        }
        else {
            text.innerText = "You do not have enough gold to buy weapon.";
        }
    }
    else {
        text.innerText = "You already have the most powerful weapon.";
        button2.innerText = "Sell weapon (15 gold)";
        button2.onclick = sellWeapon;
    }
}

function sellWeapon() {
    if (inventory.length > 1) {
        gold += 15;
        goldText.innerText = gold;
        let currentWeapon = inventory.shift();  //Local variable, for let, not the case for var...
        //Removes the first element from array and return it to the variable
        text.innerText = "You sold your " + currentWeapon + ".";
        text.innerText += "\n\nIn your inventory you have: " + inventory + ".";
    }
    else {
        text.innerText = "Don't sell your only weapon."
    }
}

function goCave() {
    // console.log("Going to cave.");
    update(locations[2]);
}

function fightSlime() {
    fighting = 0;  //Public variable
    goFight();
}

function fightBeast() {
    fighting = 1;
    goFight();
}

function fightDragon() {
    // console.log("Fighting dragon.");
    fighting = 2;
    goFight();
}
//Refactoring- change/improve already created code

function goFight() {
    update(locations[3]);
    monsterHP = monsters[fighting].health;
    monsterStats.style.display = "block";  //.style- update CSS styles in JS
    monsterName.innerText = monsters[fighting].name;
    monsterHealth.innerText = monsterHP;
}

function attack() {
    text.innerText = "The " + monsters[fighting].name + " attacks.";
    text.innerText += "\n\nYou attack it with your " + weapons[currentWeapon].name + "."
    
    if (isMonsterHit()) {
        hp -= getMonsterAttackValue(monsters[fighting].level);
    }
    else {
        text.innerText += "\n\nYou miss.";
    }
    //Added- random monster attack..., miss
    //;- doesn't affect code?

    monsterHP -= weapons[currentWeapon].power + Math.floor(Math.random() * xp) + 1;  //math.random()- bet 0-1, round down
    healthText.innerText = hp;
    monsterHealth.innerText = monsterHP;

    if (hp <= 0) {
        lose();
    }
    else if (monsterHP <= 0) {
        // if (fighting === 2) {  //===- no type conversions, ==- w/
        //     win();
        // }
        // else {
        //     defeatMonster();
        // }
        fighting === 2 ? win() : defeatMonster();
        //Ternary?/conditional operator- 1 line if-else expression
        //First is true..., useful if only 1 line after condition...
    }

    if (Math.random() <= .1 && inventory.length !== 1) {
        text.innerText += "Your " + inventory.pop() + " breaks.";  //Remove final item and return it
        currentWeapon--;
    }
}

function dodge() {
    text.innerText = "You dodge the attack from the " + monsters[fighting].name + ".";
}

function lose() {
    update(locations[5]);
}

function win() {
    dragon.style.display = "flex";
    update(locations[6]);
}

function defeatMonster() {
    gold += Math.floor(monsters[fighting].level * 6.7);
    xp += monsters[fighting].level;
    goldText.innerText = gold;
    xpText.innerText = xp;
    update(locations[4]);
}

function restart() {
    xp = 0;
    hp = 100;
    gold = 50;
    currentWeapon = 0;
    inventory = ["stick"];

    goldText.innerText = gold;
    healthText.innerText = hp;
    xpText.innerText = xp;
    // text.innerText = "Welcome to Dragon Repeller. You must defeat the dragon that is preventing people from leaving the town. You are in the town square. Where do you want to go? Use the buttons above."
    dragon.style.display = "none";
    goTown();
}

function getMonsterAttackValue(level) {
    let hit = (level*5) - (Math.floor(Math.random() * xp));
    console.log(hit);
    return hit;
}

function isMonsterHit() {
    return (Math.random() > 0.2) || (hp < 20);  //20% miss, always hit if hp<20, &&- and
}

//Mini game Easter Egg, hidden feature
function easterEgg() {
    update(locations[7]);
}

function pick2() {
    pick(2);
}

function pick8() {
    pick(8);
}

function pick(guess) {
    let numbers = [];
    //While loop
    while (numbers.length < 10) {
        numbers.push(Math.floor(Math.random() * 11));
    };

    text.innerText = "You picked " + guess + ". Here are the random numbers:\n\n";  //\n- escape character (new line)
    // text.innerText += numbers;

    //For loop
    for (let i=0; i<10; i++) {  //Let...
        text.innerText += numbers[i] + "\n"
    };

    if (numbers.indexOf(guess) !== -1) {  //-1- if not in list
        text.innerText += "\n\nRight! You win 20 gold!"
        gold += 20;
        goldText.innerText = gold;
    }
    else {
        text.innerText += "\n\nWrong! You lose 10 health!"
        hp -= 10;
        healthText.innerText = hp;
        if (hp <= 0) {
            lose();
        }
    }
}