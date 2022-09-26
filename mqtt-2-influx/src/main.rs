use std::{env, process, time::Duration};

extern crate paho_mqtt as mqtt;

const DFLT_BROKER: &str = "192.168.0.28";
const DFLT_CLIENT: &str = "influx_writer";

const TOPICS: &[&str] = &["kamstrup/activePowerPlus"];
const QOS:&[i32] = &[1];

fn subscribe_topics(cli: &mqtt::Client) {
    if let Err(e) = cli.subscribe_many(TOPICS, QOS) {
        println!("Error subscribes topics: {:?}", e);
        process::exit(1);
    }
}

fn main() {
    let host = env::args().nth(1).unwrap_or_else(||
        DFLT_BROKER.to_string()
    );
    
    // Define the set of options for the create.
    // Use an ID for a persistent session.
    let create_opts = mqtt::CreateOptionsBuilder::new()
        .server_uri(host)
        .client_id(DFLT_CLIENT.to_string())
        .finalize();
    
    // Create a client.
    let cli = mqtt::Client::new(create_opts).unwrap_or_else(|err| {
        println!("Error creating the client: {:?}", err);
        process::exit(1);
    });
    
    // Define the set of options for the connection.
    let conn_opts = mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(Duration::from_secs(20))
        .clean_session(true)
        .finalize();
    
    let rx = cli.start_consuming();

    // Connect and wait for it to complete or fail.
    if let Err(e) = cli.connect(conn_opts) {
        println!("Unable to connect:\n\t{:?}", e);
        process::exit(1);
    }
    println!("Connected to MQTT broker");

    subscribe_topics(&cli);

    for msg in rx.iter() {
        if let Some(msg) = msg {
            println!("{}", msg);
        }
    }

}
