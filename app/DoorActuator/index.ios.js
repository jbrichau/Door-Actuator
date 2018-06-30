/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 * @flow
 */

import React, {Component} from 'react';
import {AppRegistry, Alert, StyleSheet, Text, View } from 'react-native';
import { Button } from 'native-base';

export default class DoorActuator extends Component {
  calibrate() {
      const deviceId = '260028001447353236343033';
      const accessToken = '8750e519cc2d04fdf4f8c6a6205b28f57706954e';
      const body = `access_token=${encodeURIComponent(accessToken)}&args={}`;

      fetch(`https://api.particle.io/v1/devices/${deviceId}/calibrate`, {
        method: 'POST',
        headers: {
          'Accept': 'application/json',
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body
      }).catch((error) => {
        Alert.alert('An error occurred:', err);
      }).then((response) => {
        Alert.alert('Function called succesfully:', response);
      });
  };
  render() {
    return (
      <View style={styles.container}>
                <Text style={styles.welcome}>
                    Door controller 2
                </Text>
                <Button block onPress={this.calibrate}> Calibrate </Button>
                <Text style={styles.instructions}>
                    Press Cmd+R to reload,{'\n'}
                    Cmd+D or shake for dev menu
                </Text>
            </View>
      );
  }
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        justifyContent: 'center',
        alignItems: 'center',
        backgroundColor: '#F5FCFF'
    },
    welcome: {
        fontSize: 20,
        textAlign: 'center',
        margin: 10
    },
    instructions: {
        textAlign: 'center',
        color: '#333333',
        marginBottom: 5
    }
});

AppRegistry.registerComponent('DoorActuator', () => DoorActuator);
