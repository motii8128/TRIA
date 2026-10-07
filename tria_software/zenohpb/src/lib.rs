use prost::Message;
use zenoh::{
    Session, 
    bytes::Encoding, 
    pubsub::{Publisher, Subscriber},
    handlers::FifoChannelHandler, sample::Sample
};
use std::marker::PhantomData;
pub type Result<T> = std::result::Result<T, zenoh::Error>;

#[derive(Clone)]
pub struct Node 
{
    name : String,
    session :Session,
}

impl Node {
    pub async fn new(name : &str, config : zenoh::Config) -> Result<Self>
    {
        let ses = zenoh::open(config).await?;
        Ok(
            Self { name: name.to_string(), session: ses }
        )
    }

    pub fn node_name(&self) -> &str
    {
        &self.name
    }

    pub async fn create_publisher<'a, M: Message>(&'a self, topic : &'a str) -> Result<ZPublisher<'a, M>>
    {
        let inner = self.session.declare_publisher(topic).encoding(Encoding::TEXT_PLAIN).await?;

        Ok(
            ZPublisher{
                z_pub : inner,
                _marker: PhantomData
            }
        )
    }

    pub async fn create_subscription<M: Message>(&self, topic : &str)-> Result<ZSubscriber<M>>
    {
        let inner = self.session.declare_subscriber(topic).await.unwrap();

        Ok(
            ZSubscriber { z_sub: inner, _marker: PhantomData }
        )
    }
}

pub struct ZPublisher<'a, M>
{
    z_pub : Publisher<'a>,
    _marker: PhantomData<fn(M)>
}

impl<'a, M: Message> ZPublisher<'a, M> {
    pub async fn publish(&self, msg: &M) -> zenoh::Result<()>
    {
        self.z_pub.put(msg.encode_to_vec()).await
    }
}

pub struct ZSubscriber<M>
{
    z_sub : Subscriber<FifoChannelHandler<Sample>>,
    _marker : PhantomData<fn()->M>
}

impl<M: Message + Default> ZSubscriber<M> {
    pub async fn subscribe(&self) -> zenoh::Result<M>
    {
        let sample = self.z_sub.recv_async().await?;
        let bytes = sample.payload().to_bytes();

        Ok(M::decode(&*bytes)?)
    }
}