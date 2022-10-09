use std::{env, process, time::Duration};
use influxdb::{Client, Query, Timestamp, ReadQuery};
use influxdb::InfluxDbWriteable;
use chrono::{DateTime, Utc};
use std::time::{SystemTime, UNIX_EPOCH};

extern crate paho_mqtt as mqtt;

const DFLT_BROKER: &str = "192.168.0.28";
const DFLT_CLIENT: &str = "influx_writer";

const TOPICS: &[&str] = &["temperature/temp1"];
const QOS:&[i32] = &[1];

fn subscribe_topics(cli: &mqtt::Client) {
    if let Err(e) = cli.subscribe_many(TOPICS, QOS) {
        println!("Error subscribes topics: {:?}", e);
        process::exit(1);
    }
}

#[derive(InfluxDbWriteable)]
struct TempReading {
    time: DateTime<Utc>,
    temperature: f32,
}

#[tokio::main]
async fn main() {
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

    let influx = Client::new("http://192.168.0.28:8086", "house").with_auth("lars", "foobarbaz");

    // let read_query = ReadQuery::new("SELECT * FROM weather");

    // let read_result = influx.query(read_query).await;
    // if let Err(err) = &read_result { 
    //     println!("Error: {:?}", err);
    // }
    // assert!(read_result.is_ok(), "Read result was not ok");
    // println!("{}", read_result.unwrap());

    for msg in rx.iter() {
        if let Some(msg) = msg {
            println!("{}", msg);


            let start = SystemTime::now();
            let since_the_epoch = start
                 .duration_since(UNIX_EPOCH)
                .expect("Time went backwards")
                .as_millis();
            let reading = TempReading {
                time: Timestamp::Milliseconds(since_the_epoch).into(),
                temperature: msg.payload_str().parse().unwrap(),
            };

            let write_result = influx
                .query(reading.into_query("house"))
                .await;
            assert!(write_result.is_ok(), "Write result was not okay: {:?}", write_result);
            println!("Write result: {:?}", write_result);

        }
    }

}
